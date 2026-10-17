param([Parameter(ValueFromRemainingArguments = $true)][string[]]$Command)
$ErrorActionPreference = "Stop"
$repoPath = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$imageName = if ($env:IMAGE) { $env:IMAGE } else { "rci_wonik_humble:latest" }
$containerName = if ($env:CONTAINER) { $env:CONTAINER } else { "rci_wonik_humble" }
$domainId = if ($env:ROS_DOMAIN_ID) { $env:ROS_DOMAIN_ID } else { "42" }
$existing = docker container ls -a --filter "name=^/$containerName$" --format "{{.Names}}"
if ($LASTEXITCODE -ne 0) { throw "Docker is unavailable." }
if (-not $existing) {
    $runArgs = @("run", "-d", "--name", $containerName, "--shm-size=1g",
        "--mount", "type=volume,source=rci_wonik_humble_ws,target=/ros2_ws",
        "--mount", "type=bind,source=$repoPath,target=/ros2_ws/src/RCI_wonik_robotics2",
        "-e", "ROS_DOMAIN_ID=$domainId", $imageName)
    & docker @runArgs | Out-Null
} else {
    docker start $containerName | Out-Null
}
if ($LASTEXITCODE -ne 0) { throw "Cannot start $containerName." }
if (-not $Command) { $Command = @("bash") }
$execArgs = @("exec")
if (-not [Console]::IsInputRedirected) { $execArgs += "-it" }
$execArgs += @($containerName, "/usr/local/bin/wonik-command")
& docker @execArgs @Command
exit $LASTEXITCODE
