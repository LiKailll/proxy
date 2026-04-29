@echo off
setlocal enabledelayedexpansion

:: 获取脚本所在的目录
set "SCRIPT_DIR=%~dp0"
echo %SCRIPT_DIR%
:: 切换到项目根目录
cd /d "%~dp0.."
:: 设置项目目录变量
set "PROJECT_DIR=%cd%"
echo %PROJECT_DIR%

:: 定义四个版本的 Makefile 文件名
set "proxy_V1=proxy_3slot"
set "proxy_V2=proxy_5slot"
set "proxy_V3=proxy_8slot"
set "proxy_V4=proxy_fudanwei\proxy_8slot"

:: 编译每个版本的代理项目
for /l %%i in (1, 1, 4) do (
    :: 调用 make 命令编译项目
    echo %PROJECT_DIR%\!proxy_V%%i!
    cd "%PROJECT_DIR%\!proxy_V%%i!"
    make
    cd %PROJECT_DIR%

    :: 检查 make 命令是否成功执行
    if !errorlevel! neq 0 (
        echo *** Error during make process for version !i! [!errorlevel!]
        set "EXIT_CODE=!errorlevel!"
        goto :EXIT
    )
)

:: 输出成功信息
echo All build processes completed successfully.
set "EXIT_CODE=0"

:EXIT
:: 设置退出代码并退出脚本
endlocal & set "EXIT_CODE=%EXIT_CODE%"
exit /b %EXIT_CODE%