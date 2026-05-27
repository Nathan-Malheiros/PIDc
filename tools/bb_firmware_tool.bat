@echo off
setlocal
:: Carrega o perfil ESP-IDF (instalacao EIM/Espressif) e abre a ferramenta.
PowerShell.exe -NoProfile -ExecutionPolicy Bypass -Command ^
  "$prof='C:\Espressif\tools\Microsoft.v6.0.1.PowerShell_profile.ps1'; if(Test-Path $prof){ . $prof }; & '%~dp0bb_firmware_tool.ps1'"
endlocal
