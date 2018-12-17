set MATLAB=E:\App

cd .

if "%1"=="" ("E:\App\bin\win64\gmake"  -f CF_INS.mk all) else ("E:\App\bin\win64\gmake"  -f CF_INS.mk %1)
@if errorlevel 1 goto error_exit

exit /B 0

:error_exit
echo The make command returned an error of %errorlevel%
An_error_occurred_during_the_call_to_make
