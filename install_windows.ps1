
Write-Host
Write-Host "+------------------------------+"
Write-Host "|    FicTrac install script    |"
Write-Host "+------------------------------+"
Write-Host

$MSYS_DIR = Read-Host -Prompt "Enter full path to MSYS install directory (e.g. C:\msys64)"
$MSYS_BIN_DIR = "$MSYS_DIR\mingw64\bin"
if (Test-Path -Path $MSYS_BIN_DIR) {
    Write-Host "Found MSYS bin dir at: $MSYS_BIN_DIR"
}
else {
    Write-Host "Uh oh, couldn't find the MSYS bin dir at: $MSYS_BIN_DIR"
    exit
}

$USER_PATH = [Environment]::GetEnvironmentVariable("Path", "User")
if (-Not $USER_PATH.contains("$MSYS_BIN_DIR")) {
    $Env:PATH += ";$MSYS_BIN_DIR"    # set locally in script
    [Environment]::SetEnvironmentVariable("Path", $USER_PATH + ";$MSYS_BIN_DIR", "User")     # set permanently
    Write-Host "MSYS bin dir added to Path variable"
}


Write-Host
Write-Host "+-- Installing dependencies ---+"
Write-Host

Write-Host "Please copy and execute the following command in the MSYS console:"
Write-Host
Write-Host "pacman -Sy mingw-w64-x86_64-gcc   \"
Write-Host "           mingw-w64-x86_64-make  \"
Write-Host "           mingw-w64-x86_64-nlopt \"
Write-Host "           mingw-w64-x86_64-boost \"
Write-Host "           mingw-w64-x86_64-ffmpeg\"
Write-Host "           mingw-w64-x86_64-opencv"
Write-Host
Write-Host "Close the MSYS console when all commands have completed successully"

Start-Process "$MSYS_DIR\msys2_shell.cmd" -Wait


Write-Host
Write-Host "+-- Creating build directory --+"
Write-Host
$FICTRAC_DIR = Split-Path -Parent $MyInvocation.MyCommand.Path
cd $FICTRAC_DIR    # make sure we are in fictrac dir
if (Test-Path -Path "./build") {
    Write-Host "Removing existing build dir"
    Remove-Item -Recurse "./build"
}
$null = New-Item -ItemType Directory -Path "./build"
if (Test-Path -Path "./build") {
	Write-Host "Created build dir"
	cd "./build"
}
else {
	Write-Host "Uh oh, something went wrong attempting to create the build dir!"
	exit
}


Write-Host
Write-Host "+-- Generating build files ----+"
Write-Host
cmake -G "MinGW Makefiles" ..


Write-Host
Write-Host "+-- Building FicTrac ----------+"
Write-Host
cmake --build . --config Release --parallel 4 --clean-first


cd ..
if (Test-Path "./bin/fictrac.exe" -PathType Leaf) {
	Write-Host
	Write-Host "FicTrac built successfully!"
	Write-Host
}
else {
	Write-Host
	Write-Host "Hmm... something seems to have gone wrong - can't find FicTrac executable."
	Write-Host
}
