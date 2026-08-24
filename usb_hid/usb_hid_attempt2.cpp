#include <SDL.h>

// g++ block_visuomotor_task_usb_hid.cpp -std=c++17 $(sdl2-config --cflags --libs) -o block_visuomotor_task_usb_hid.exe
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <random>
#include <string>

// g++ block_visuomotor_task_usb_hid (2).cpp -std=c++17 -IC:\msys64\ucrt64\include\SDL2 -LC:\msys64\ucrt64\lib -lmingw32 -lSDL2main -lSDL2 -o block_visuomotor_task_usb_hid.exe

// ------------------------------------------------------------
// Block-based visuomotor joystick task for Windows.
//
// Block 1: 5 automatic trials with normal cursor mapping.
// Block 2: 5 controlled trials with inverted cursor mapping.
//
// Input: USB HID joystick read directly through SDL2.
// Preferred device: VID 068E, PID 019B.
//
// SDL joystick axes are converted from -32768..32767 to 0..1023
// so the rest of the task uses the same coordinate system as the
// original Arduino implementation.
//
// Joystick button 0 is read but is not used to choose condition.
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

constexpr Uint16 TARGET_VENDOR_ID = 0x068E;
constexpr Uint16 TARGET_PRODUCT_ID = 0x019B;

int hidAxisToArduinoRange(Sint16 axisValue)
{
    const int shifted = static_cast<int>(axisValue) + 32768;
    return static_cast<int>(
        (static_cast<long long>(shifted) * JOYSTICK_MAX) / 65535
    );
}

SDL_Joystick* openUsbHidJoystick()
{
    const int joystickCount = SDL_NumJoysticks();

    if (joystickCount <= 0)
    {
        std::cerr << "No SDL joystick devices were detected.\n";
        return nullptr;
    }

    std::cout << "Detected " << joystickCount << " joystick device(s):\n";

    int selectedIndex = -1;

    for (int i = 0; i < joystickCount; ++i)
    {
        const Uint16 vendor = SDL_JoystickGetDeviceVendor(i);
        const Uint16 product = SDL_JoystickGetDeviceProduct(i);
        const char* name = SDL_JoystickNameForIndex(i);

        std::cout
            << "  [" << i << "] "
            << (name != nullptr ? name : "Unknown joystick")
            << " | VID: 0x" << std::hex << vendor
            << " | PID: 0x" << product
            << std::dec << "\n";

        if (vendor == TARGET_VENDOR_ID && product == TARGET_PRODUCT_ID)
        {
            selectedIndex = i;
        }
    }

    if (selectedIndex < 0)
    {
        std::cerr
            << "Could not find the expected USB HID joystick "
            << "VID 068E / PID 019B.\n";
        return nullptr;
    }

    SDL_Joystick* joystick = SDL_JoystickOpen(selectedIndex);

    if (joystick == nullptr)
    {
        std::cerr
            << "Could not open USB HID joystick: "
            << SDL_GetError() << "\n";
        return nullptr;
    }

    if (SDL_JoystickNumAxes(joystick) < 2)
    {
        std::cerr << "Joystick does not expose at least two axes.\n";
        SDL_JoystickClose(joystick);
        return nullptr;
    }

    std::cout
        << "Using joystick: "
        << SDL_JoystickName(joystick)
        << " | axes=" << SDL_JoystickNumAxes(joystick)
        << " | buttons=" << SDL_JoystickNumButtons(joystick)
        << "\n";

    return joystick;
}

void readUsbHidJoystick(
    SDL_Joystick* device,
    JoystickState& joystick
)
{
    SDL_JoystickUpdate();

    joystick.x = hidAxisToArduinoRange(
        SDL_JoystickGetAxis(device, 0)
    );

    joystick.y = hidAxisToArduinoRange(
        SDL_JoystickGetAxis(device, 1)
    );

    joystick.button =
        SDL_JoystickNumButtons(device) > 0
            ? static_cast<int>(SDL_JoystickGetButton(device, 0))
            : 0;
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

    for (int i = 0; i < 8; ++i)
    {
        const double angle =
            -M_PI / 2.0 + i * (2.0 * M_PI / 8.0);

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
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK) != 0)
    {
        std::cerr
            << "SDL initialization failed: "
            << SDL_GetError()
            << "\n";

        return 1;
    }

    SDL_JoystickEventState(SDL_ENABLE);

    SDL_Joystick* hidJoystick = openUsbHidJoystick();

    if (hidJoystick == nullptr)
    {
        SDL_Quit();
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

        SDL_JoystickClose(hidJoystick);
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
        SDL_JoystickClose(hidJoystick);
        SDL_Quit();
        return 1;
    }

    std::ofstream resultsFile(
        "block_visuomotor_results_usb_hid.csv"
    );

    if (!resultsFile)
    {
        std::cerr
            << "Could not create block_visuomotor_results_usb_hid.csv\n";

        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_JoystickClose(hidJoystick);
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

            if (event.type == SDL_JOYDEVICEREMOVED &&
                event.jdevice.which == SDL_JoystickInstanceID(hidJoystick))
            {
                std::cerr << "Joystick disconnected.\n";
                running = false;
            }
        }

        readUsbHidJoystick(hidJoystick, joystick);

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
    SDL_JoystickClose(hidJoystick);
    SDL_Quit();

    std::cout
        << "Task complete. Results saved to "
        << "block_visuomotor_results_usb_hid.csv\n";

    return 0;
}
