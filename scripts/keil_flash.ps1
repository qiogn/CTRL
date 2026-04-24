param(
  [string]$ProjectFile = "C:\2025E\STM32\CTRL\MDK-ARM\CTRL.uvprojx",
  [string]$KeilPath = "D:\Keil_v5\UV4"
)

$uvisionCom = "$KeilPath\uVision.com"
$logFile = "C:\2025E\STM32\CTRL\MDK-ARM\flash.log"

Write-Host "Flashing with Keil via uVision.com..."
Write-Host "Project: $ProjectFile"

# Build and flash
& $uvisionCom -b $ProjectFile -j0 -d -o $logFile

if ($LASTEXITCODE -eq 0) {
  Write-Host "Flash completed successfully"
  if (Test-Path $logFile) {
    Get-Content $logFile | Select-Object -Last 50
  }
} else {
  Write-Host "Flash failed with exit code: $LASTEXITCODE"
  if (Test-Path $logFile) {
    Get-Content $logFile
  }
}
