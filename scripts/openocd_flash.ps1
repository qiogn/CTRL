param(
  [string]$ImageFile = (Join-Path $PSScriptRoot "..\MDK-ARM\CTRL\CTRL.axf")
)

function Get-OpenOcdPath {
  $candidates = @(
    'C:\Users\SJN31\AppData\Local\Microsoft\WinGet\Packages\xpack-dev-tools.openocd-xpack_Microsoft.Winget.Source_8wekyb3d8bbwe\xpack-openocd-0.12.0-7\bin\openocd.exe'
  )

  foreach ($candidate in $candidates) {
    if (Test-Path $candidate) {
      return $candidate
    }
  }

  $command = Get-Command openocd -ErrorAction SilentlyContinue
  if ($command) {
    return $command.Source
  }

  throw 'openocd.exe not found'
}

$openocd = Get-OpenOcdPath
$resolvedImage = (Resolve-Path $ImageFile).Path
$resolvedImageOpenOcd = $resolvedImage -replace '\\', '/'
$scriptDir = Join-Path (Split-Path $openocd -Parent | Split-Path -Parent) 'openocd\scripts'

Write-Host "Flashing with OpenOCD: $resolvedImage"
& $openocd `
  -s $scriptDir `
  -f interface/stlink.cfg `
  -f target/stm32f1x.cfg `
  -c "program $resolvedImageOpenOcd verify reset exit"

if ($LASTEXITCODE -ne 0) {
  exit $LASTEXITCODE
}
