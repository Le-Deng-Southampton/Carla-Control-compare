from queue import Empty, Queue

import carla
import numpy as np
import pygame

from control import create_tracking_controller
from error_providers import create_error_provider
from road_planning.route_planner import build_shaped_route
from Speed_Planing import CurvatureSpeedPlanner, SpeedPlannerConfig

from .logging import append_step_data, build_step_snapshot, log_step
from .metrics import create_metrics
from .termination import CollisionZeroSpeedTerminator


ROUTE_CLOSE_DISTANCE_M = 8.0


class SimpleHUD:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.font = pygame.font.Font(None, 28)
        self.info_text = []

    def update(
        self,
        vehicle,
        control,
        controller_name,
        target_speed_mps=None,
        speed_planner_mode=None,
    ):
        vel = vehicle.get_velocity()
        speed = 3.6 * np.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)
        transform = vehicle.get_transform()
        location = transform.location
        target_speed_kmh = 3.6 * target_speed_mps if target_speed_mps is not None else None
        speed_error_kmh = speed - target_speed_kmh if target_speed_kmh is not None else None
        mode_label = speed_planner_mode or "unknown"

        self.info_text = [
            f"Controller: {controller_name.upper()}",
            f"Speed: {speed:5.1f} km/h",
            f"Target Speed: {target_speed_kmh:5.1f} km/h" if target_speed_kmh is not None else "Target Speed: n/a",
            f"Speed Error: {speed_error_kmh:+5.1f} km/h" if speed_error_kmh is not None else "Speed Error: n/a",
            f"Speed Mode: {mode_label}",
            f"Steer: {control.steer:+.2f}",
            f"Throttle: {control.throttle:.2f}",
            f"Brake: {control.brake:.2f}",
            f"Location: ({location.x:.1f}, {location.y:.1f}, {location.z:.1f})",
        ]

    def render(self, display):
        v_offset = 4
        for text in self.info_text:
            surf = self.font.render(text, True, (255, 255, 255))
            shadow = self.font.render(text, True, (0, 0, 0))
            display.blit(shadow, (9, v_offset + 1))
            display.blit(surf, (8, v_offset))
            v_offset += 22


def compute_lookahead(base_lookahead, speed_mps):
    return max(3.0, base_lookahead, 0.35 * speed_mps)


def route_curvature(route_trace, route_index):
    if route_index <= 0 or route_index >= len(route_trace) - 1:
        return 0.0

    prev_wp, _ = route_trace[route_index - 1]
    curr_wp, _ = route_trace[route_index]
    next_wp, _ = route_trace[route_index + 1]

    x1, y1 = prev_wp.transform.location.x, prev_wp.transform.location.y
    x2, y2 = curr_wp.transform.location.x, curr_wp.transform.location.y
    x3, y3 = next_wp.transform.location.x, next_wp.transform.location.y

    cross = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1)
    a = np.sqrt((x3 - x2) ** 2 + (y3 - y2) ** 2)
    b = np.sqrt((x3 - x1) ** 2 + (y3 - y1) ** 2)
    c = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    denom = a * b * c
    if denom < 1e-6 or abs(cross) < 1e-6:
        return 0.0

    radius = denom / abs(2.0 * cross)
    curvature = 1.0 / radius if cross > 0 else -1.0 / radius
    return float(np.clip(curvature, -0.2, 0.2))


def build_curvature_preview(route_trace, reference_index, preview_steps=(0, 5, 10, 15), curvature_fn=route_curvature):
    curvatures = []
    last_index = max(0, len(route_trace) - 2)
    for step in preview_steps:
        preview_index = min(max(0, reference_index + step), last_index)
        curvatures.append(curvature_fn(route_trace, preview_index))
    return curvatures


def build_controller_curvature_preview(
    route_trace,
    reference_index,
    speed_mps,
    horizon,
    dt,
    curvature_fn=route_curvature,
):
    distance_per_step_m = max(float(speed_mps) * float(dt), 1.0)
    preview_steps = [int(round(step * distance_per_step_m)) for step in range(max(int(horizon), 1))]
    return build_curvature_preview(
        route_trace,
        reference_index,
        preview_steps=preview_steps,
        curvature_fn=curvature_fn,
    )


def build_speed_planner(args):
    return CurvatureSpeedPlanner(
        SpeedPlannerConfig(
            base_target_speed_kmh=args.target_speed,
            min_turn_speed_kmh=args.speed_planner_min_turn_speed,
            max_lateral_accel=args.speed_planner_max_lateral_accel,
            max_accel=args.speed_planner_max_accel,
            max_decel=args.speed_planner_max_decel,
            lateral_error_warning=args.speed_planner_lateral_error_warning,
            lateral_error_critical=args.speed_planner_lateral_error_critical,
            heading_error_warning_rad=np.radians(args.speed_planner_heading_error_warning),
            heading_error_critical_rad=np.radians(args.speed_planner_heading_error_critical),
            lateral_error_rate_warning=args.speed_planner_lateral_error_rate_warning,
            lateral_error_rate_critical=args.speed_planner_lateral_error_rate_critical,
            heading_error_rate_warning_rad=np.radians(args.speed_planner_heading_error_rate_warning),
            heading_error_rate_critical_rad=np.radians(args.speed_planner_heading_error_rate_critical),
            lateral_error_rate_activation=args.speed_planner_lateral_error_rate_activation,
            heading_error_rate_activation_rad=np.radians(args.speed_planner_heading_error_rate_activation),
            error_rate_filter_alpha=args.speed_planner_error_rate_alpha,
            recovery_hold_steps=args.speed_planner_recovery_hold_steps,
            entry_max_speed_kmh=args.speed_planner_entry_max_speed,
            entry_curvature_threshold=args.speed_planner_entry_curvature_threshold,
            entry_full_cap_curvature=args.speed_planner_entry_full_cap_curvature,
        )
    )


def spawn_camera(world, blueprint_library, vehicle, display_width, display_height):
    camera_bp = blueprint_library.find("sensor.camera.rgb")
    camera_bp.set_attribute("image_size_x", str(display_width))
    camera_bp.set_attribute("image_size_y", str(display_height))
    camera_bp.set_attribute("fov", "110")
    camera_transform = carla.Transform(carla.Location(x=-5.5, z=2.5), carla.Rotation(pitch=-15))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    image_queue = Queue()

    def camera_callback(image):
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((display_height, display_width, 4))
        image_queue.put(array[:, :, :3])

    camera.listen(camera_callback)
    return camera, image_queue


def drain_latest_image(image_queue):
    latest_image = None
    try:
        while True:
            latest_image = image_queue.get_nowait()
    except Empty:
        pass
    return latest_image


def render_frame(
    display,
    image_queue,
    hud,
    vehicle,
    control,
    controller_name,
    clock,
    target_speed_mps=None,
    speed_planner_mode=None,
):
    latest_image = drain_latest_image(image_queue)
    if latest_image is not None:
        surface = pygame.surfarray.make_surface(latest_image.swapaxes(0, 1))
        display.blit(surface, (0, 0))
    else:
        display.fill((0, 0, 0))

    hud.update(
        vehicle,
        control,
        controller_name,
        target_speed_mps=target_speed_mps,
        speed_planner_mode=speed_planner_mode,
    )
    hud.render(display)
    pygame.display.flip()
    clock.tick_busy_loop(60)


def create_display(title, display_width, display_height):
    pygame.init()
    display = pygame.display.set_mode((display_width, display_height), pygame.HWSURFACE | pygame.DOUBLEBUF)
    pygame.display.set_caption(title)
    return display


def configure_world(world, control_dt):
    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = control_dt
    world.apply_settings(settings)
    return original_settings


def get_vehicle_blueprint(world):
    blueprint_library = world.get_blueprint_library()
    vehicle_bps = blueprint_library.filter("model3")
    if not vehicle_bps:
        raise RuntimeError("No 'model3' vehicle blueprint found.")
    return blueprint_library, vehicle_bps[0]


def resolve_route_setup(world, args, clamp_spawn_index, rng):
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        raise RuntimeError("No spawn points available.")

    if args.spawn_index is None:
        spawn_index = rng.randrange(len(spawn_points))
    else:
        spawn_index = clamp_spawn_index(args.spawn_index, spawn_points)

    if args.destination_index is None:
        destination_candidates = [idx for idx in range(len(spawn_points)) if idx != spawn_index]
        if not destination_candidates:
            raise RuntimeError("At least two spawn points are required to build a route.")
        destination_index = rng.choice(destination_candidates)
    else:
        destination_index = clamp_spawn_index(args.destination_index, spawn_points)
        if destination_index == spawn_index and len(spawn_points) > 1:
            destination_index = (spawn_index + 1) % len(spawn_points)

    spawn_point = spawn_points[spawn_index]
    fallback_destination = spawn_points[destination_index].location
    route_trace, route_features = build_shaped_route(
        world.get_map(),
        spawn_point.location,
        fallback_destination,
        args.route_resolution,
        args.route_min_length_m,
        args.route_max_waypoints,
        ROUTE_CLOSE_DISTANCE_M,
        route_shape=args.route_shape,
        length_tolerance=args.route_length_tolerance,
    )
    destination = route_trace[-1][0].transform.location
    return spawn_index, destination_index, spawn_point, destination, route_trace, route_features


def create_experiment_runtime(controller_name, vehicle, args):
    return {
        "controller": create_tracking_controller(controller_name, vehicle, args),
        "error_provider": create_error_provider(args.error_provider, args),
        "speed_planner": build_speed_planner(args),
    }


def apply_controller_step(
    controller,
    vehicle,
    target_waypoint,
    route_trace,
    target_index,
    reference_waypoint,
    reference_index,
    planned_target_speed_mps=None,
    tracking_errors=None,
):
    curvature_index = reference_index if reference_index is not None else target_index
    curvature = route_curvature(route_trace, curvature_index)
    if getattr(controller, "supports_curvature_sequence", False):
        velocity = vehicle.get_velocity()
        speed_mps = np.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
        curvature = build_controller_curvature_preview(
            route_trace,
            curvature_index,
            speed_mps=speed_mps,
            horizon=getattr(controller, "horizon", 1),
            dt=getattr(controller, "dt", 0.05),
        )
    return controller.run_step(
        vehicle,
        target_waypoint,
        curvature,
        reference_waypoint=reference_waypoint,
        planned_target_speed_mps=planned_target_speed_mps,
        tracking_errors=tracking_errors,
    )


def select_route_target(route_trace, vehicle_loc, last_index, look_ahead):
    start = max(0, last_index - 5)
    end = min(len(route_trace), last_index + 120)
    closest_index = start
    closest_distance = float("inf")

    for idx in range(start, end):
        distance = route_trace[idx][0].transform.location.distance(vehicle_loc)
        if distance < closest_distance:
            closest_index = idx
            closest_distance = distance

    target_index = closest_index
    while target_index < len(route_trace) - 1:
        distance = route_trace[target_index][0].transform.location.distance(vehicle_loc)
        if distance >= look_ahead:
            break
        target_index += 1

    target_waypoint, road_option = route_trace[target_index]
    return target_waypoint, road_option, target_index, closest_index, closest_distance


def run_controller_lap(controller_name, world, blueprint_library, vehicle_bp, spawn_point, route_trace, args, display, display_width, display_height):
    vehicle = None
    camera = None
    collision_sensor = None
    collision_count = [0]
    rows = []
    positions_x, positions_y = [], []
    metrics = create_metrics()
    route_index = 0
    previous_steer = 0.0
    zero_speed_terminator = CollisionZeroSpeedTerminator(
        timeout_seconds=args.collision_zero_speed_timeout,
        speed_threshold_mps=args.collision_zero_speed_threshold,
    )

    try:
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        vehicle.set_autopilot(False)
        collision_bp = blueprint_library.find("sensor.other.collision")
        collision_sensor = world.spawn_actor(collision_bp, carla.Transform(), attach_to=vehicle)

        def on_collision(event):
            collision_count[0] += 1

        collision_sensor.listen(on_collision)
        runtime = create_experiment_runtime(controller_name, vehicle, args)
        camera, image_queue = spawn_camera(world, blueprint_library, vehicle, display_width, display_height)
        hud = SimpleHUD(display_width, display_height)
        clock = pygame.time.Clock()

        print(f"Starting {controller_name.upper()} lap from the same spawn point.")

        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return rows, positions_x, positions_y, metrics, True
                if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    return rows, positions_x, positions_y, metrics, True

            world.tick()
            vehicle_loc = vehicle.get_location()
            speed = vehicle.get_velocity()
            speed_ms = np.sqrt(speed.x ** 2 + speed.y ** 2 + speed.z ** 2)
            dyn_lookahead = compute_lookahead(args.look_ahead, speed_ms)

            target_waypoint, road_option, route_index, closest_route_index, route_error = select_route_target(
                route_trace,
                vehicle_loc,
                route_index,
                dyn_lookahead,
            )
            reference_waypoint = route_trace[closest_route_index][0]
            tracking_errors = runtime["error_provider"].compute(
                vehicle,
                target_waypoint,
                route_trace,
                closest_route_index,
                reference_waypoint=reference_waypoint,
            )
            if args.speed_planner_mode == "off":
                target_speed_ms = args.target_speed / 3.6
                runtime["speed_planner"].last_risk = 0.0
                runtime["speed_planner"].last_reason = "fixed_throttle_brake"
            else:
                curvature_preview = build_curvature_preview(route_trace, closest_route_index)
                target_speed_ms = runtime["speed_planner"].plan_speed_mps(
                    curvature_preview,
                    getattr(args, "control_dt", 0.05),
                    lateral_error_m=tracking_errors["e_y"],
                    heading_error_rad=tracking_errors["e_psi"],
                )
            control = apply_controller_step(
                runtime["controller"],
                vehicle,
                target_waypoint,
                route_trace,
                route_index,
                reference_waypoint,
                closest_route_index,
                planned_target_speed_mps=target_speed_ms,
                tracking_errors=tracking_errors,
            )
            vehicle.apply_control(control)

            snapshot = build_step_snapshot(
                vehicle,
                control,
                target_speed_ms,
                previous_steer,
                tracking_errors,
                speed_plan_risk=runtime["speed_planner"].last_risk,
                speed_plan_reason=runtime["speed_planner"].last_reason,
            )
            previous_steer = control.steer
            sim_time = world.get_snapshot().timestamp.elapsed_seconds
            append_step_data(
                rows,
                positions_x,
                positions_y,
                metrics,
                controller_name,
                args,
                (route_index, closest_route_index, route_error, road_option),
                snapshot,
                control,
                sim_time,
            )
            render_frame(
                display,
                image_queue,
                hud,
                vehicle,
                control,
                controller_name,
                clock,
                target_speed_mps=target_speed_ms,
                speed_planner_mode=args.speed_planner_mode,
            )
            log_step(controller_name, route_trace, route_index, snapshot, control)

            if zero_speed_terminator.update(snapshot["speed"], sim_time, collision_count[0]):
                print(
                    f"{controller_name.upper()} lap terminated: collision followed by "
                    f"speed <= {args.collision_zero_speed_threshold:.2f} m/s for "
                    f"{args.collision_zero_speed_timeout:.1f} s."
                )
                return rows, positions_x, positions_y, metrics, False

            if route_index >= len(route_trace) - 2:
                print(f"{controller_name.upper()} lap finished.")
                return rows, positions_x, positions_y, metrics, False
    finally:
        if collision_sensor is not None:
            collision_sensor.destroy()
        if camera is not None:
            camera.destroy()
        if vehicle is not None:
            vehicle.destroy()
