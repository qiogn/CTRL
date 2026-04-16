param(
  [string]$ProjectFile = (Join-Path $PSScriptRoot "..\MDK-ARM\CTRL.uvprojx")
)

function Get-UV4Path {
  $keys = @(
    'HKLM:\SOFTWARE\WOW6432Node\Keil\Products\MDK',
    'HKLM:\SOFTWARE\Keil\Products\MDK',
    'HKCU:\SOFTWARE\Keil\Products\MDK'
  )

  foreach ($key in $keys) {
    if (Test-Path $key) {
      $armPath = (Get-ItemProperty $key).Path
      if ($armPath) {
        $candidate = Join-Path (Split-Path $armPath -Parent) 'UV4\UV4.exe'
        if (Test-Path $candidate) {
          return $candidate
        }
      }
    }
  }

  throw 'UV4.exe not found'
}

$uv4 = Get-UV4Path
$resolvedProject = (Resolve-Path $ProjectFile).Path

Write-Host "Building with Keil: $resolvedProject"
& $uv4 -b $resolvedProject -j0

if ($LASTEXITCODE -ne 0) {
  exit $LASTEXITCODE
}
