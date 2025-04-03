@echo off
echo Finding USB device with VID:PID 0483:374B...

:: Run PowerShell to find the correct BusID
for /f "tokens=1" %%A in ('powershell -Command " (usbipd list | Select-String -Pattern '(\d+-\d+).*0483:374B').Matches.Groups[1].Value.Trim()"') do (
    set BUSID=%%A
)

:: Check if BUSID is found
if "%BUSID%"=="" (
    echo No matching USB device found!
    pause
    exit /b
)

echo Found USB device at BusID: %BUSID%
echo Attaching to WSL...
usbipd attach --wsl --busid %BUSID%

echo Binding USB device...
usbipd bind --busid %BUSID%

echo Done.
pause
