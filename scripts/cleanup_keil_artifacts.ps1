# Cleanup Keil generated build artifacts
# Removes common build outputs from MDK-ARM/CTRL (e.g. .o, .d, .crf, .axf, .hex, .map, .lnp, .htm)
$root = Split-Path -Parent $MyInvocation.MyCommand.Definition
$mdkDir = Join-Path $root "..\MDK-ARM\CTRL"
if (-Not (Test-Path $mdkDir)) {
  Write-Host "MDK-ARM/CTRL path not found: $mdkDir"
  exit 0
}

$patterns = '*.o','*.d','*.crf','*.axf','*.hex','*.map','*.lnp','*.htm','*.build_log.htm','*.sct','*.dep'
foreach ($p in $patterns) {
  Get-ChildItem -Path $mdkDir -Filter $p -File -ErrorAction SilentlyContinue | ForEach-Object {
    try {
      Remove-Item -LiteralPath $_.FullName -Force -ErrorAction Stop
      Write-Host "Deleted: $($_.FullName)"
    }
    catch {
      Write-Warning "Failed to delete $($_.FullName): $($_.Exception.Message)"
    }
  }
}

Write-Host "Cleanup complete. Please check MDK-ARM/CTRL directory to confirm."