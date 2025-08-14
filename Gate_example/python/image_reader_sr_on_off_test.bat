@echo off

setlocal enabledelayedexpansion

set /A MIN_ITER=5
set /A MAX_ITER=8

set /A MIN_WAIT=8
set /A MAX_WAIT=12

set /A MAX_PASS=1000

for /l %%x in (1,1, %MAX_PASS%) do (
    set /A "ITER=!random! %% (!MAX_ITER! - !MIN_ITER! + 1) + !MIN_ITER!"
    set /A "WAIT=!random! %% (!MAX_WAIT! - !MIN_WAIT! + 1) + !MIN_WAIT!"
    python image_reader_sr.py -i!ITER!
	echo | set /p=Iteration %%x from %MAX_PASS% completed
    timeout !WAIT!
)
