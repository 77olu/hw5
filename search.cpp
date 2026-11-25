/*
 * STUDENT CODE
 */

#include "search.h"

#include <algorithm>
#include <vector>

Search::Search(Mothership &mothership)
    : mothership(mothership)
{
}

SearchResult Search::findShipwreck()
{
  const int count = mothership.getRobotCount();

  if (count <= 0)
  {
    return SearchResult{false, Location{}};
  }

  // Furthest safe segment for each robot.
  std::vector<int> lastSafe(count, TETHER_LENGTH);

  // Find the nearest abyss on each path.
  for (const Location &abyss : mothership.getAbysses())
  {
    lastSafe[abyss.robot] =
        std::min(lastSafe[abyss.robot], abyss.segment - 1);
  }

  // Discoveries on each path are naturally recorded in sorted order,
  // because robots only move forward.
  std::vector<std::vector<int>> debris(count);

  auto signalStrength = [&](int robot) -> int
  {
    const int position = mothership.getRobotPosition(robot);
    const int low = position - SIGNAL_RANGE;
    const int high = position + SIGNAL_RANGE;

    const int paths[3] = {
        robot,
        (robot + count - 1) % count,
        (robot + 1) % count};

    int strength = 0;

    for (int i = 0; i < 3; ++i)
    {
      // Avoid counting the same path twice when N is 1 or 2.
      bool duplicate = false;
      for (int j = 0; j < i; ++j)
      {
        if (paths[i] == paths[j])
        {
          duplicate = true;
          break;
        }
      }

      if (duplicate)
      {
        continue;
      }

      const std::vector<int> &discoveries = debris[paths[i]];

      auto first = std::lower_bound(
          discoveries.begin(), discoveries.end(), low);
      auto afterLast = std::upper_bound(
          discoveries.begin(), discoveries.end(), high);

      strength += static_cast<int>(afterLast - first);
    }

    return strength;
  };

  std::vector<int> active;
  active.reserve(count);

  for (int robot = 0; robot < count; ++robot)
  {
    if (mothership.getRobotPosition(robot) < lastSafe[robot])
    {
      active.push_back(robot);
    }
  }

  std::vector<int> scores(count, 0);

  while (!active.empty())
  {
    // Snapshot scores before sorting so comparisons stay consistent.
    for (int robot : active)
    {
      scores[robot] = signalStrength(robot);
    }

    std::sort(active.begin(), active.end(),
              [&](int left, int right)
              {
                if (scores[left] != scores[right])
                {
                  return scores[left] > scores[right];
                }
                return left < right;
              });

    for (int robot : active)
    {
      if (mothership.getRobotPosition(robot) >= lastSafe[robot])
      {
        continue;
      }

      if (mothership.getActiveRobot() != robot)
      {
        mothership.switchToRobot(robot);
      }

      // Explore a baseline distance on every turn.
      const int baseline = 2 * SIGNAL_RANGE;
      const int maximum = 8 * SIGNAL_RANGE;
      int distanceThisTurn = 0;

      while (mothership.getRobotPosition(robot) < lastSafe[robot] &&
             distanceThisTurn < maximum)
      {
        // Continue past the baseline while nearby debris
        // still makes this path promising.
        if (distanceThisTurn >= baseline &&
            signalStrength(robot) == 0)
        {
          break;
        }

        const int position = mothership.getRobotPosition(robot);
        const int safeDistance = lastSafe[robot] - position;

        const int requested = std::min(
            MAX_SEGMENTS_PER_DRIVE,
            std::min(safeDistance, maximum - distanceThisTurn));

        const DriveResult result =
            mothership.driveActiveRobot(requested);

        distanceThisTurn += result.segmentsMoved;

        const int newPosition =
            mothership.getRobotPosition(robot);

        if (result.reason == DriveStopReason::ShipwreckFound)
        {
          return SearchResult{true, Location{robot, newPosition}};
        }

        if (result.reason == DriveStopReason::DebrisFound)
        {
          debris[robot].push_back(newPosition);
        }
      }
    }

    // Remove robots that cannot advance any further.
    active.erase(
        std::remove_if(
            active.begin(), active.end(),
            [&](int robot)
            {
              return mothership.getRobotPosition(robot) >=
                     lastSafe[robot];
            }),
        active.end());
  }

  return SearchResult{false, Location{}};
}