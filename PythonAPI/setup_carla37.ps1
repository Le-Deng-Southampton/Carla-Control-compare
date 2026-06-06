$ErrorActionPreference = "Stop"

$projectDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$wheel = Join-Path $projectDir "carla\dist\carla-0.9.14-cp37-cp37m-win_amd64.whl"
$commonScript = Join-Path $projectDir "scripts\common.ps1"

. $commonScript
$conda = Get-CondaCommand


Push-Location $projectDir
try {
    $existing = & $conda env list | Select-String -Pattern "^\s*carla37\s"
    if ($existing) {
        Invoke-CondaCommand $conda install -y -n carla37 --override-channels -c conda-forge --solver=libmamba python=3.7 openssl=1.1.1 libdeflate=1.14 libtiff=4.4 pip numpy=1.21 scipy=1.7 matplotlib=3.5 pillow=9.2 networkx=2.6 shapely=1.8 future distro
    }
    else {
        Invoke-CondaCommand $conda create -y -n carla37 --override-channels -c conda-forge --solver=libmamba python=3.7 openssl=1.1.1 libdeflate=1.14 libtiff=4.4 pip numpy=1.21 scipy=1.7 matplotlib=3.5 pillow=9.2 networkx=2.6 shapely=1.8 future distro
    }

    Invoke-CondaCommand $conda run -n carla37 python -m pip install pygame==2.6.1
    Invoke-CondaCommand $conda run -n carla37 python -m pip install $wheel
}
finally {
    Pop-Location
}
