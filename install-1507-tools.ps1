# install-1507-tools.ps1
# Installs the Team 1507 Subsystem Tools VS Code extension locally.
# Run once per machine from the project root:
#   .\install-1507-tools.ps1
# Then reload VS Code (Ctrl+Shift+P → "Developer: Reload Window").

$source = Join-Path $PSScriptRoot ".vscode\team1507-tools"
$dest   = Join-Path $env:USERPROFILE ".vscode\extensions\team1507.subsystem-tools-1.0.0"

if (-not (Test-Path $source)) {
    Write-Error "Extension source not found at: $source"
    exit 1
}

Write-Host "Installing Team 1507 Subsystem Tools..."
if (Test-Path $dest) {
    Remove-Item $dest -Recurse -Force
}
Copy-Item -Path $source -Destination $dest -Recurse
Write-Host ""
Write-Host "Done! Reload VS Code to activate:"
Write-Host "  Ctrl+Shift+P -> Developer: Reload Window"
Write-Host ""
Write-Host "Usage: right-click any folder in the Explorer -> 'New Subsystem1507...'"
