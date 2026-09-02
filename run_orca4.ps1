<#
.SYNOPSIS
    Run the orca4 simulation from Windows.

.DESCRIPTION
    orca4 is a ROS 2 / Gazebo project and only runs on Linux, so the simulation
    itself lives in WSL. This is a thin wrapper: it hands off to ~/orca4_run.sh
    inside the WSL distro and forwards the exit code back to Windows.

    The Gazebo and RViz windows are drawn through WSLg and appear as ordinary
    Windows windows.

.PARAMETER Headless
    Skip the Gazebo UI and RViz. Much faster, and the mission still runs.

.PARAMETER NoMission
    Bring the simulation up and leave it running, without sending the mission.

.PARAMETER SoftwareGl
    Force software rendering. Try this if Gazebo opens black or not at all.

.PARAMETER Bag
    Record a rosbag of the interesting topics.

.PARAMETER TimeoutSeconds
    How long to wait for the sim to become ready before giving up. Default 240.

.PARAMETER Distro
    WSL distro to use. Default Ubuntu-22.04.

.EXAMPLE
    .\run_orca4.ps1
    Full run with the Gazebo and RViz windows, then the default mission.

.EXAMPLE
    .\run_orca4.ps1 -Headless
    No UI. Fastest way to watch the mission execute in the console.

.EXAMPLE
    .\run_orca4.ps1 -NoMission
    Just bring the simulation up and leave it running.

.NOTES
    Press Ctrl+C to shut the whole simulation down.
    Logs land in \\wsl.localhost\<distro>\home\<you>\.orca4_run\
#>
[CmdletBinding()]
param(
    [switch] $Headless,
    [switch] $NoMission,
    [switch] $SoftwareGl,
    [switch] $Bag,
    [int]    $TimeoutSeconds = 0,
    [string] $Distro = 'Ubuntu-22.04'
)

$ErrorActionPreference = 'Stop'

function Write-Step { param([string] $Text) Write-Host "==> $Text" -ForegroundColor Cyan }
function Write-Bad  { param([string] $Text) Write-Host "!!  $Text" -ForegroundColor Red }

# --- checks ---------------------------------------------------------------

if (-not (Get-Command wsl.exe -ErrorAction SilentlyContinue)) {
    Write-Bad 'wsl.exe not found. Is WSL installed?'
    exit 1
}

# `wsl -l -q` returns UTF-16, so test the distro by using it rather than by
# parsing the list.
$null = wsl.exe -d $Distro -- true 2>$null
if ($LASTEXITCODE -ne 0) {
    Write-Bad "WSL distro '$Distro' is not available."
    Write-Host '    Distros on this machine:'
    wsl.exe -l -v
    Write-Host "    Pass a different one with:  .\run_orca4.ps1 -Distro <name>"
    exit 1
}

$null = wsl.exe -d $Distro --cd ~ -- test -x orca4_run.sh 2>$null
if ($LASTEXITCODE -ne 0) {
    Write-Bad "~/orca4_run.sh not found (or not executable) in '$Distro'."
    Write-Host '    That script is what actually launches the simulation.'
    exit 1
}

# --- build the argument list ----------------------------------------------

$scriptArgs = @()
if ($Headless)   { $scriptArgs += '--headless' }
if ($NoMission)  { $scriptArgs += '--no-mission' }
if ($SoftwareGl) { $scriptArgs += '--software-gl' }
if ($Bag)        { $scriptArgs += '--bag' }
if ($TimeoutSeconds -gt 0) { $scriptArgs += @('--timeout', "$TimeoutSeconds") }

Write-Step "Starting orca4 in WSL ($Distro)"
if ($scriptArgs.Count -gt 0) {
    Write-Host "    options: $($scriptArgs -join ' ')"
}
if (-not $Headless) {
    Write-Host '    Gazebo and RViz will open as separate windows (via WSLg).'
}
Write-Host '    Press Ctrl+C to stop everything.'
Write-Host ''

# --- run ------------------------------------------------------------------

# --cd ~ so `bash orca4_run.sh` resolves without quoting a Linux path through
# PowerShell. The Linux script cd's to the colcon workspace itself, which it
# must do -- ORB_SLAM2 loads its vocabulary via a relative path.
wsl.exe -d $Distro --cd ~ -- bash orca4_run.sh @scriptArgs
$rc = $LASTEXITCODE

Write-Host ''
if ($rc -eq 0) {
    Write-Step 'Finished.'
} else {
    Write-Bad "Exited with code $rc."
    Write-Host '    The Linux-side log for this run is under:'
    Write-Host "    \\wsl.localhost\$Distro\home\<you>\.orca4_run\"
}

exit $rc
