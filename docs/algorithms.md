# Algorithm pseudocode

The controller combines frontier selection, global A* planning, and reactive path following. Algorithms 1–6 below are adapted from the numbered pseudocode in the [project report](report/mobile-robotics-report.pdf#page=6), with names and formatting adjusted for readability. They describe the report's intended design; the [implementation notes](implementation-notes.md) explain differences and edge cases in the preserved source.

The report describes path following in prose rather than a numbered algorithm. The final section adds a separate pseudocode summary based on that description and the original follower source.

- [Frontier exploration: Algorithms 1–3](#frontier-exploration)
- [Global path planning: Algorithms 4–6](#global-path-planning)
- [Reactive path follower: additional summary](#reactive-path-follower)

## Frontier exploration

**Source:** report Section 4.1, printed pages 4–6; [frontier selector implementation](../src/frontier_explorer/frontier_explorer/frontier_explorer_node.py).

A frontier is a free map cell next to unknown space. Eight-connected BFS groups these cells into clusters. Each cluster contributes one representative goal, and distance determines which goal is tried first.

### Algorithm 1: Frontier exploration main loop

```text
procedure FrontierExplorer:
    current_goal = NONE
    selected_rank = 0
    cycle_count = 0

    while exploration is running:
        map, pose = latest map and robot pose in the map frame
        if map is missing or pose is unavailable:
            continue

        candidates = BuildFrontierCandidates(map)          # Algorithm 2
        if candidates is empty:
            stop exploration

        ranked = RankCandidates(candidates, pose)          # Algorithm 3

        if current_goal has been reached:
            current_goal = NONE
            selected_rank += 1
            cycle_count = 0
        else if next frontier has been requested:
            current_goal = NONE
            selected_rank += 1
        else if current_goal is invalid in the current map:
            current_goal = NONE

        if current_goal is NONE and cooldown has elapsed:
            if selected_rank >= number of ranked candidates:
                selected_rank = 0
                cycle_count += 1
                if cycle_count >= max_full_cycles:
                    stop exploration

            current_goal = ranked[selected_rank].goal
            publish current_goal on /frontier_goal
```

A goal deadline is also described in the report's high-level overview. In the follower, expiration can produce a `/frontier_next` request, which feeds the next-frontier branch above. Exhausting retry cycles is a stopping condition, not proof that every reachable area has been mapped.

### Algorithm 2: Build frontier candidates

```text
procedure BuildFrontierCandidates(map):
    classify map cells as free, occupied, or unknown
    radius_cells = ceil((robot_radius + safety_margin) / map_resolution)
    inflate occupied cells by radius_cells

    frontier = free cells with at least one unknown eight-neighbor
    clusters = BFS connected components of frontier, using eight-neighbors
    discard clusters smaller than min_cluster_size

    candidates = empty list
    for each cluster C:
        median_row = median of row coordinates in C
        median_col = median of column coordinates in C
        representative = cell in C nearest to (median_row, median_col)
        goal = convert representative to world coordinates
        append (goal, size = number of cells in C) to candidates

    return candidates
```

Inflation supplies a clearance buffer. The implementation can use the inflated grid for frontier extraction and uses it for current-goal validity checks. The representative remains a cell belonging to its cluster; it is not simply the potentially off-cluster median coordinate.

### Algorithm 3: Rank frontier candidates

```text
procedure RankCandidates(candidates, robot_pose):
    for each candidate:
        candidate.distance = EuclideanDistance(robot_pose, candidate.goal)

    sort candidates by (distance ascending, cluster size descending)
    return candidates
```

Distance has priority. Cluster size breaks distance ties; this is not a weighted information-gain score.

## Global path planning

**Source:** report Section 4.2, printed pages 6–9; [A* planner implementation](../src/astar_planner/astar_planner/frontier_astar_planner.py).

The planner uses an inflated occupancy grid and considers nearby free goal cells in increasing distance from the requested goal. A* uses eight-connected moves, costs of `1` for cardinal moves and `sqrt(2)` for diagonals, and a Euclidean heuristic. Search is limited by the configured number of expansions.

### Algorithm 4: A* planner event handling

```text
subscribe to /frontier_goal, /map, /map_updates, /odom, and /replan
current_goal = NONE
last_plan_time = negative infinity

while ROS is running:
    when /frontier_goal is received:
        current_goal = received goal
        PlanToGoal(current_goal)                           # Algorithm 5

    when /replan is received:
        if current_goal exists and replan cooldown has elapsed:
            PlanToGoal(current_goal)

    when /map_updates is received:
        if map-update replanning is enabled and its rate limit allows:
            if current_goal exists:
                PlanToGoal(current_goal)
```

The planning routine records timing for cooldown checks. A map or pose must be available before planning can proceed. The source takes the planner pose from `/odom`; its assumption about map and odometry coordinates is documented under [coordinate frames](implementation-notes.md#coordinate-frames).

### Algorithm 5: Plan-to-goal pipeline

```text
procedure PlanToGoal(goal_world):
    robot_position = latest position from /odom
    start = WorldToGrid(robot_position)
    goal = WorldToGrid(goal_world)

    occupied = OccupiedMask(map)
    blocked = Inflate(occupied, inflation_radius)
    start = FixStartIfBlocked(start, blocked, start_search_radius)
    goal_candidates = FixGoalIfBlocked(goal, blocked, goal_search_radius)

    selected, success = SearchBestPath(                    # Algorithm 6
        start, goal_candidates, blocked,
        goal_candidate_attempts, max_expansions
    )
    if success is false:
        return                                            # No full or partial path

    if selected is a partial path:
        update failure counters
        if failures are repeated:
            publish /frontier_next

    selected = ShortcutSmoothing(selected, blocked)
    selected = MovingAverageSmoothing(selected)
    publish GridPathToWorldPath(selected) on /planned_path
```

The report's pipeline above shows partial-path failure counting. **The preserved implementation differs:** it resets `failed_plan_count` when either a full path or a usable partial path is selected. Its failure counter advances when planning produces no usable path or fails prerequisite checks. The source also converts the shortcut grid path to world coordinates before applying its moving-average filter.

Shortcut smoothing removes intermediate waypoints when Bresenham line-of-sight checks find a clear connection. The moving average then adjusts waypoint positions. The original source does not collision-check the final averaged path, and its partial-path branch has a logging exception after publication; see [partial-path fallback](implementation-notes.md#partial-path-fallback) and [path assumptions](implementation-notes.md#path-and-obstacle-assumptions).

### Algorithm 6: A* search over multiple goal candidates

```text
procedure SearchBestPath(start, goal_candidates, blocked,
                         goal_candidate_attempts, max_expansions):
    best_full = NONE
    best_partial = NONE
    best_score = positive infinity

    for each goal in the first min(goal_candidate_attempts,
                                  number of goal_candidates) candidates:
        path, reached, score = AStar(start, goal, blocked, max_expansions)

        if reached:
            best_full = path
            break
        else if score < best_score:
            best_score = score
            best_partial = path

    if best_full exists:
        return best_full, true
    else if best_partial exists:
        return best_partial, true
    else:
        return NONE, false
```

For an incomplete search, A* reconstructs a route to the expanded node nearest the candidate goal. The partial-path score is the remaining Euclidean grid distance to that candidate. A full route to the first reachable candidate takes priority over partial routes; otherwise, the closest partial endpoint is selected. Here, `success = true` means a path was selected and can include a partial route; it does not mean the frontier was reached. The source requires at least two path points before accepting a route for publication.

## Reactive path follower

**Additional summary, not a seventh numbered algorithm from the report.** Based on report Section 4.3, printed page 9, and [the original follower](../src/pathfollower_new555.py).

The follower has three states: `FOLLOW_PATH`, `AVOID_TURN`, and `AVOID_DRIVE`. It follows waypoints in ordinary motion and uses current laser sectors to choose a side, turn away from a blocked front sector, and look for a waypoint it can rejoin.

```text
on /frontier_goal:
    store frontier goal and restart its deadline

on a /planned_path with at least two poses:
    store waypoints and reset waypoint index
    reset progress timers, replan counter, and avoidance state

at each control tick:
    if robot pose is unavailable:
        return
    if no usable path exists:
        publish zero velocity
        return

    target = frontier goal if available, otherwise the last waypoint
    goal_distance = distance from robot to target

    if frontier deadline expired and target is not reached:
        request next frontier; stop and clear the current path
        return
    if goal distance improved by the progress threshold:
        record the new best distance and progress time
    if overall timeout since accepting the path expired and target is not reached:
        request next frontier; stop and clear the current path
        return
    if no progress for the stall timeout and target is not reached:
        RequestReplan("no progress")
        restart the stall timer

    if target is within goal tolerance:
        publish zero velocity and mark the path inactive
        return

    if current waypoint is within its tolerance:
        advance waypoint index
        if no waypoints remain:
            RequestReplan("path ended before frontier goal")
            retain the final waypoint index
        return

    if a fresh scan is available:
        if front is blocked, state is FOLLOW_PATH, and avoidance cooldown elapsed:
            choose turn direction from mean left/right sector clearances
            state = AVOID_TURN
            start the avoidance timer
        if avoidance has lasted longer than twice the avoidance timeout:
            RequestReplan("avoidance did not recover")

    if scan is fresh and state is in {AVOID_TURN, AVOID_DRIVE}:
        front_clear = front distance >= safety_distance + clearance_hysteresis
        if state is AVOID_TURN:
            command a turn with no forward motion
            if front_clear and minimum turn duration elapsed:
                if a visible rejoin waypoint exists:
                    resume FOLLOW_PATH at that waypoint
                else:
                    state = AVOID_DRIVE
        else if state is AVOID_DRIVE:
            if front is not clear:
                state = AVOID_TURN; command a turn with no forward motion
            else:
                command forward motion with a turn bias
                if a visible rejoin waypoint exists:
                    resume FOLLOW_PATH at that waypoint
    else:
        compute waypoint heading error and distance
        reduce forward motion as heading error increases
        compute and clamp normalized differential-drive commands
        apply the brief forward-commit setting after rejoining

    rate-limit commands, scale to configured robot velocity limits,
    and publish TwistStamped on /cmd_vel

procedure RequestReplan(reason):
    if replan cooldown has not elapsed:
        return
    publish true on /replan and increment replan count
    if replan count reaches its configured limit:
        request next frontier; stop and clear the current path
```

The rejoin search checks a bounded number of upcoming waypoints for a permitted bearing and sufficient laser clearance. Its helper can also advance from the current index to the following waypoint. Rejoining starts an avoidance cooldown and a short forward-commit interval to reduce repeated switching.

This is a readable summary of the existing behavior, not a corrected controller. Missing or stale scans bypass avoidance and allow ordinary tracking. A replan request can clear the path while the calling control tick continues, creating the indexing edge case documented in [reactive control and recovery](implementation-notes.md#reactive-control-and-recovery). Replacement paths reset recovery counters, and several deadlines use wall time. These caveats also apply when interpreting the pseudocode.
