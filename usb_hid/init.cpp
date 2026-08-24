#include <windows.h>
#include <mmsystem.h>
#include <stdio.h>

#pragma comment(lib, "winmm.lib")

int main(void)
{
    UINT numJoysticks = joyGetNumDevs();

    printf("Windows reports %u joystick slots.\n", numJoysticks);

    JOYINFOEX joyInfo;
    joyInfo.dwSize = sizeof(JOYINFOEX);
    joyInfo.dwFlags = JOY_RETURNALL;

    while (1)
    {
        MMRESULT result = joyGetPosEx(JOYSTICKID1, &joyInfo);

        if (result == JOYERR_NOERROR)
        {
            printf(
                "X: %5lu | Y: %5lu | Z: %5lu | Buttons: 0x%08lX\r",
                joyInfo.dwXpos,
                joyInfo.dwYpos,
                joyInfo.dwZpos,
                joyInfo.dwButtons
            );

            fflush(stdout);
        }
        else
        {
            printf("\nCould not read joystick. Error code: %u\n", result);
            break;
        }

        Sleep(20);
    }

    return 0;
}