/*
 * This file is part of the AzerothCore Project. See AUTHORS file for Copyright information
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Affero General Public License as published by the
 * Free Software Foundation; either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Affero General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "GameTime.h"
#include "Timer.h"
#include <atomic>

namespace GameTime
{
    using namespace std::chrono;

    Seconds const StartTime = GetEpochTime();

    // Atomic variables to prevent race conditions during concurrent access
    std::atomic<Seconds::rep> GameTimeAtomic{GetEpochTime().count()};
    std::atomic<Milliseconds::rep> GameMSTimeAtomic{0};

    std::atomic<SystemTimePoint::rep> GameTimeSystemPointAtomic{SystemTimePoint::min().time_since_epoch().count()};
    std::atomic<TimePoint::rep> GameTimeSteadyPointAtomic{TimePoint::min().time_since_epoch().count()};

    Seconds GetStartTime()
    {
        return StartTime;
    }

    Seconds GetGameTime()
    {
        return Seconds{GameTimeAtomic.load(std::memory_order_relaxed)};
    }

    Milliseconds GetGameTimeMS()
    {
        return Milliseconds{GameMSTimeAtomic.load(std::memory_order_relaxed)};
    }

    SystemTimePoint GetSystemTime()
    {
        auto count = GameTimeSystemPointAtomic.load(std::memory_order_relaxed);
        return SystemTimePoint{SystemTimePoint::duration{count}};
    }

    TimePoint Now()
    {
        auto count = GameTimeSteadyPointAtomic.load(std::memory_order_relaxed);
        return TimePoint{TimePoint::duration{count}};
    }

    Seconds GetUptime()
    {
        return GetGameTime() - StartTime;
    }

    void UpdateGameTimers()
    {
        // Use relaxed memory ordering since we don't need strict ordering between these updates
        // and the performance benefit is significant for time-critical operations
        GameTimeAtomic.store(GetEpochTime().count(), std::memory_order_relaxed);
        GameMSTimeAtomic.store(GetTimeMS().count(), std::memory_order_relaxed);
        GameTimeSystemPointAtomic.store(system_clock::now().time_since_epoch().count(), std::memory_order_relaxed);
        GameTimeSteadyPointAtomic.store(steady_clock::now().time_since_epoch().count(), std::memory_order_relaxed);
    }
}
