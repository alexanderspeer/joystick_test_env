#include <SDL.h>
#include <windows.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <string>

//g++ block_visuomotor_task_windows_fixed.cpp -std=c++17 $(sdl2-config --cflags --libs) -o block_visuomotor_task_windows.exe

// ------------------------------------------------------------
// Block-based visuomotor joystick task for Windows.
//
// Block 1: 5 automatic trials with normal cursor mapping.
// Block 2: 5 controlled trials with inverted cursor mapping.
//
// Arduino serial format:
//     x,y,sw
//
// Arduino baud rate:
//     Serial.begin(9600);
//
// The joystick switch is read but is not used to choose condition.
// ------------------------------------------------------------

constexpr int WINDOW_WIDTH = 1000;
constexpr int WINDOW_HEIGHT = 750;

constexpr int JOYSTICK_MIN = 0;
constexpr int JOYSTICK_MAX = 1023;

constexpr int CURSOR_RADIUS = 8;
constexpr int TARGET_RADIUS = 24;
constexpr int CENTER_RADIUS = 18;
constexpr int TARGET_DISTANCE = 245;

constexpr int TRIALS_PER_BLOCK = 5;
constexpr int NUMBER_OF_BLOCKS = 2;

constexpr double CENTER_HOLD_SECONDS = 3.0;
constexpr double WARNING_SECONDS = 0.5;
constexpr double PRE_TARGET_DELAY_SECONDS = 0.5;
constexpr double FEEDBACK_SECONDS = 0.5;

constexpr int CENTER_TOLERANCE = 35;
constexpr int JOYSTICK_DEADZONE = 8;

enum class Condition
{
    Automatic,
    Controlled
};

enum class TrialPhase
{
    WaitingForCenter,
    HoldingCenter,
    Warning,
    PreTargetDelay,
    TargetVisible,
    Feedback,
    Finished
};

struct JoystickState
{
    int x = 512;
    int y = 512;
    int button = 1;
};

using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

std::string serialBuffer;

double secondsSince(const TimePoint& start)
{
    return std::chrono::duration<double>(Clock::now() - start).count();
}

double millisecondsBetween(const TimePoint& start, const TimePoint& end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

const char* conditionName(Condition condition)
{
    return condition == Condition::Automatic
        ? "automatic"
        : "controlled";
}

HANDLE openSerialPort(const std::string& portName)
{
    const std::string fullPortName = "\\\\.\\" + portName;

    HANDLE serialHandle = CreateFileA(
        fullPortName.c_str(),
        GENERIC_READ,
        0,
        nullptr,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        nullptr
    );

    if (serialHandle == INVALID_HANDLE_VALUE)
    {
        std::cerr
            << "Could not open " << portName
            << ". Windows error code: "
            << GetLastError()
            << "\\n";

        return INVALID_HANDLE_VALUE;
    }

    DCB serialParameters{};
    serialParameters.DCBlength = sizeof(serialParameters);

    if (!GetCommState(serialHandle, &serialParameters))
    {
        std::cerr << "Could not read serial-port settings.\\n";
        CloseHandle(serialHandle);
        return INVALID_HANDLE_VALUE;
    }

    serialParameters.BaudRate = CBR_9600;
    serialParameters.ByteSize = 8;
    serialParameters.StopBits = ONESTOPBIT;
    serialParameters.Parity = NOPARITY;
    serialParameters.fBinary = TRUE;
    serialParameters.fDtrControl = DTR_CONTROL_ENABLE;
    serialParameters.fRtsControl = RTS_CONTROL_ENABLE;

    if (!SetCommState(serialHandle, &serialParameters))
    {
        std::cerr << "Could not configure serial port.\\n";
        CloseHandle(serialHandle);
        return INVALID_HANDLE_VALUE;
    }

    COMMTIMEOUTS timeouts{};
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutConstant = 0;
    timeouts.ReadTotalTimeoutMultiplier = 0;

    if (!SetCommTimeouts(serialHandle, &timeouts))
    {
        std::cerr << "Could not configure serial timeouts.\\n";
        CloseHandle(serialHandle);
        return INVALID_HANDLE_VALUE;
    }

    PurgeComm(serialHandle, PURGE_RXCLEAR | PURGE_TXCLEAR);
    return serialHandle;
}

bool parseJoystickLine(const std::string& line, JoystickState& joystick)
{
    std::stringstream stream(line);

    std::string xText;
    std::string yText;
    std::string buttonText;

    if (!std::getline(stream, xText, ',') ||
        !std::getline(stream, yText, ',') ||
        !std::getline(stream, buttonText, ','))
    {
        return false;
    }

    try
    {
        const int newX = std::stoi(xText);
        const int newY = std::stoi(yText);
        const int newButton = std::stoi(buttonText);

        if (newX < JOYSTICK_MIN || newX > JOYSTICK_MAX ||
            newY < JOYSTICK_MIN || newY > JOYSTICK_MAX ||
            (newButton != 0 && newButton != 1))
        {
            return false;
        }

        joystick.x = newX;
        joystick.y = newY;
        joystick.button = newButton;
        return true;
    }
    catch (...)
    {
        return false;
    }
}

void readSerialData(
    HANDLE serialHandle,
    JoystickState& joystick
)
{
    char incoming[256];
    DWORD bytesRead = 0;

    while (true)
    {
        if (!ReadFile(
                serialHandle,
                incoming,
                sizeof(incoming),
                &bytesRead,
                nullptr
            ))
        {
            break;
        }

        if (bytesRead == 0)
        {
            break;
        }

        for (DWORD i = 0; i < bytesRead; ++i)
        {
            const char character = incoming[i];

            if (character == '\n')
            {
                parseJoystickLine(serialBuffer, joystick);
                serialBuffer.clear();
            }
            else if (character != '\r')
            {
                serialBuffer += character;

                if (serialBuffer.size() > 100)
                {
                    serialBuffer.clear();
                }
            }
        }
    }
}

int mapValue(int value, int outputMinimum, int outputMaximum)
{
    value = std::clamp(value, JOYSTICK_MIN, JOYSTICK_MAX);

    const double fraction =
        static_cast<double>(value - JOYSTICK_MIN) /
        static_cast<double>(JOYSTICK_MAX - JOYSTICK_MIN);

    return outputMinimum +
           static_cast<int>(
               fraction * (outputMaximum - outputMinimum)
           );
}

void drawFilledCircle(
    SDL_Renderer* renderer,
    int centerX,
    int centerY,
    int radius
)
{
    for (int y = -radius; y <= radius; ++y)
    {
        const int halfWidth =
            static_cast<int>(
                std::sqrt(radius * radius - y * y)
            );

        SDL_RenderDrawLine(
            renderer,
            centerX - halfWidth,
            centerY + y,
            centerX + halfWidth,
            centerY + y
        );
    }
}

void drawCross(
    SDL_Renderer* renderer,
    int centerX,
    int centerY,
    int halfSize
)
{
    SDL_RenderDrawLine(
        renderer,
        centerX - halfSize,
        centerY,
        centerX + halfSize,
        centerY
    );

    SDL_RenderDrawLine(
        renderer,
        centerX,
        centerY - halfSize,
        centerX,
        centerY + halfSize
    );
}

double distanceBetween(int x1, int y1, int x2, int y2)
{
    const double dx = static_cast<double>(x1 - x2);
    const double dy = static_cast<double>(y1 - y2);

    return std::sqrt(dx * dx + dy * dy);
}

std::array<SDL_Point, 8> makeTargetPositions()
{
    std::array<SDL_Point, 8> targets{};

    const int centerX = WINDOW_WIDTH / 2;
    const int centerY = WINDOW_HEIGHT / 2;

    constexpr double PI = 3.14159265358979323846;

    for (int i = 0; i < 8; ++i)
    {
        const double angle =
            -PI / 2.0 + i * (2.0 * PI / 8.0);

        targets[i].x =
            centerX +
            static_cast<int>(TARGET_DISTANCE * std::cos(angle));

        targets[i].y =
            centerY +
            static_cast<int>(TARGET_DISTANCE * std::sin(angle));
    }

    return targets;
}

int main()
{
    const std::string portName = "COM3";

    HANDLE serialHandle = openSerialPort(portName);

    if (serialHandle == INVALID_HANDLE_VALUE)
    {
        std::cerr
            << "Check the serial port name and close "
            << "the Arduino Serial Monitor.\n";
        return 1;
    }

    Sleep(2000);
    PurgeComm(serialHandle, PURGE_RXCLEAR | PURGE_TXCLEAR);

    if (SDL_Init(SDL_INIT_VIDEO) != 0)
    {
        std::cerr
            << "SDL initialization failed: "
            << SDL_GetError()
            << "\n";

        CloseHandle(serialHandle);
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow(
        "Block-Based Visuomotor Task",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        WINDOW_WIDTH,
        WINDOW_HEIGHT,
        SDL_WINDOW_SHOWN
    );

    if (window == nullptr)
    {
        std::cerr
            << "Window creation failed: "
            << SDL_GetError()
            << "\n";

        CloseHandle(serialHandle);
        SDL_Quit();
        return 1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(
        window,
        -1,
        SDL_RENDERER_ACCELERATED |
        SDL_RENDERER_PRESENTVSYNC
    );

    if (renderer == nullptr)
    {
        std::cerr
            << "Renderer creation failed: "
            << SDL_GetError()
            << "\n";

        SDL_DestroyWindow(window);
        CloseHandle(serialHandle);
        SDL_Quit();
        return 1;
    }

    std::ofstream resultsFile(
        "block_visuomotor_results_windows.csv"
    );

    if (!resultsFile)
    {
        std::cerr
            << "Could not create block_visuomotor_results_windows.csv\n";

        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        CloseHandle(serialHandle);
        SDL_Quit();
        return 1;
    }

    resultsFile
        << "trial,block,condition,target_index,"
        << "reaction_time_ms,movement_time_ms,"
        << "total_time_ms,success\n";

    std::random_device randomDevice;
    std::mt19937 generator(randomDevice());
    std::uniform_int_distribution<int> targetDistribution(0, 7);

    const auto targets = makeTargetPositions();

    JoystickState joystick;

    TrialPhase phase = TrialPhase::WaitingForCenter;
    TimePoint phaseStart = Clock::now();
    TimePoint targetAppearanceTime{};
    TimePoint movementOnsetTime{};

    int currentBlock = 0;
    int trialWithinBlock = 0;
    int overallTrial = 0;
    int targetIndex = targetDistribution(generator);

    bool movementStarted = false;
    bool running = true;

    const int centerX = WINDOW_WIDTH / 2;
    const int centerY = WINDOW_HEIGHT / 2;

    double baselineX = 512.0;
    double baselineY = 512.0;
    int baselineSamples = 0;

    while (running)
    {
        SDL_Event event{};

        while (SDL_PollEvent(&event))
        {
            if (event.type == SDL_QUIT)
            {
                running = false;
            }

            if (event.type == SDL_KEYDOWN &&
                event.key.keysym.sym == SDLK_ESCAPE)
            {
                running = false;
            }
        }

        readSerialData(serialHandle, joystick);

        if (phase == TrialPhase::Finished)
        {
            running = false;
            continue;
        }

        const Condition condition =
            currentBlock == 0
                ? Condition::Automatic
                : Condition::Controlled;

        int rawX = joystick.x;
        int rawY = joystick.y;

        if (condition == Condition::Controlled)
        {
            rawX = JOYSTICK_MAX - rawX;
            rawY = JOYSTICK_MAX - rawY;
        }

        const int cursorX = mapValue(
            rawX,
            CURSOR_RADIUS,
            WINDOW_WIDTH - CURSOR_RADIUS
        );

        const int cursorY = mapValue(
            rawY,
            CURSOR_RADIUS,
            WINDOW_HEIGHT - CURSOR_RADIUS
        );

        const bool cursorCentered =
            distanceBetween(
                cursorX,
                cursorY,
                centerX,
                centerY
            ) <= CENTER_TOLERANCE;

        const bool joystickMoved =
            std::abs(
                joystick.x -
                static_cast<int>(baselineX)
            ) > JOYSTICK_DEADZONE ||
            std::abs(
                joystick.y -
                static_cast<int>(baselineY)
            ) > JOYSTICK_DEADZONE;

        switch (phase)
        {
            case TrialPhase::WaitingForCenter:
            {
                if (cursorCentered)
                {
                    phase = TrialPhase::HoldingCenter;
                    phaseStart = Clock::now();

                    baselineX = joystick.x;
                    baselineY = joystick.y;
                    baselineSamples = 1;
                }
                break;
            }

            case TrialPhase::HoldingCenter:
            {
                if (!cursorCentered)
                {
                    phase = TrialPhase::WaitingForCenter;
                    phaseStart = Clock::now();
                    break;
                }

                ++baselineSamples;

                baselineX +=
                    (joystick.x - baselineX) /
                    static_cast<double>(baselineSamples);

                baselineY +=
                    (joystick.y - baselineY) /
                    static_cast<double>(baselineSamples);

                if (secondsSince(phaseStart) >= CENTER_HOLD_SECONDS)
                {
                    phase = TrialPhase::Warning;
                    phaseStart = Clock::now();
                }
                break;
            }

            case TrialPhase::Warning:
            {
                if (!cursorCentered)
                {
                    phase = TrialPhase::WaitingForCenter;
                    phaseStart = Clock::now();
                    break;
                }

                if (secondsSince(phaseStart) >= WARNING_SECONDS)
                {
                    phase = TrialPhase::PreTargetDelay;
                    phaseStart = Clock::now();
                }
                break;
            }

            case TrialPhase::PreTargetDelay:
            {
                if (!cursorCentered)
                {
                    phase = TrialPhase::WaitingForCenter;
                    phaseStart = Clock::now();
                    break;
                }

                if (secondsSince(phaseStart) >= PRE_TARGET_DELAY_SECONDS)
                {
                    phase = TrialPhase::TargetVisible;
                    targetAppearanceTime = Clock::now();
                    movementStarted = false;
                }
                break;
            }

            case TrialPhase::TargetVisible:
            {
                if (!movementStarted && joystickMoved)
                {
                    movementStarted = true;
                    movementOnsetTime = Clock::now();
                }

                const SDL_Point& target = targets[targetIndex];

                const bool reachedTarget =
                    distanceBetween(
                        cursorX,
                        cursorY,
                        target.x,
                        target.y
                    ) <= TARGET_RADIUS;

                if (reachedTarget)
                {
                    const TimePoint targetReachedTime =
                        Clock::now();

                    double reactionTimeMs = -1.0;
                    double movementTimeMs = -1.0;

                    if (movementStarted)
                    {
                        reactionTimeMs =
                            millisecondsBetween(
                                targetAppearanceTime,
                                movementOnsetTime
                            );

                        movementTimeMs =
                            millisecondsBetween(
                                movementOnsetTime,
                                targetReachedTime
                            );
                    }

                    const double totalTimeMs =
                        millisecondsBetween(
                            targetAppearanceTime,
                            targetReachedTime
                        );

                    resultsFile
                        << overallTrial + 1 << ","
                        << currentBlock + 1 << ","
                        << conditionName(condition) << ","
                        << targetIndex << ","
                        << reactionTimeMs << ","
                        << movementTimeMs << ","
                        << totalTimeMs << ","
                        << 1
                        << "\n";

                    resultsFile.flush();

                    std::cout
                        << "Trial " << overallTrial + 1
                        << " | Block " << currentBlock + 1
                        << " | " << conditionName(condition)
                        << " | RT: " << reactionTimeMs << " ms"
                        << " | MT: " << movementTimeMs << " ms\n";

                    ++overallTrial;
                    ++trialWithinBlock;

                    if (trialWithinBlock >= TRIALS_PER_BLOCK)
                    {
                        ++currentBlock;
                        trialWithinBlock = 0;

                        if (currentBlock == 1)
                        {
                            std::cout
                                << "\nStarting controlled block. "
                                << "Cursor mapping is now inverted.\n\n";
                        }
                    }

                    if (currentBlock >= NUMBER_OF_BLOCKS)
                    {
                        phase = TrialPhase::Finished;
                    }
                    else
                    {
                        targetIndex = targetDistribution(generator);
                        phase = TrialPhase::Feedback;
                        phaseStart = Clock::now();
                    }
                }

                break;
            }

            case TrialPhase::Feedback:
            {
                if (secondsSince(phaseStart) >= FEEDBACK_SECONDS)
                {
                    phase = TrialPhase::WaitingForCenter;
                    phaseStart = Clock::now();
                }
                break;
            }

            case TrialPhase::Finished:
                break;
        }

        SDL_SetRenderDrawColor(renderer, 235, 235, 235, 255);
        SDL_RenderClear(renderer);

        SDL_SetRenderDrawColor(renderer, 20, 20, 20, 255);
        drawCross(renderer, centerX, centerY, 12);

        if (phase == TrialPhase::WaitingForCenter ||
            phase == TrialPhase::HoldingCenter ||
            phase == TrialPhase::Warning ||
            phase == TrialPhase::PreTargetDelay)
        {
            SDL_SetRenderDrawColor(renderer, 130, 130, 130, 255);
            drawFilledCircle(
                renderer,
                centerX,
                centerY,
                CENTER_RADIUS
            );

            SDL_SetRenderDrawColor(renderer, 20, 20, 20, 255);
            drawCross(renderer, centerX, centerY, 12);
        }

        if (phase == TrialPhase::Warning)
        {
            SDL_SetRenderDrawColor(renderer, 245, 200, 25, 255);
            drawFilledCircle(
                renderer,
                centerX,
                centerY,
                CENTER_RADIUS + 14
            );

            SDL_SetRenderDrawColor(renderer, 20, 20, 20, 255);
            drawCross(renderer, centerX, centerY, 12);
        }

        if (phase == TrialPhase::TargetVisible)
        {
            const SDL_Point& target = targets[targetIndex];

            // Same target color in both blocks.
            SDL_SetRenderDrawColor(renderer, 20, 190, 70, 255);

            drawFilledCircle(
                renderer,
                target.x,
                target.y,
                TARGET_RADIUS
            );
        }

        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        drawFilledCircle(
            renderer,
            cursorX,
            cursorY,
            CURSOR_RADIUS
        );

        SDL_RenderPresent(renderer);
    }

    resultsFile.close();
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    CloseHandle(serialHandle);
    SDL_Quit();

    std::cout
        << "Task complete. Results saved to "
        << "block_visuomotor_results_windows.csv\n";

    return 0;
}
