# PlanSys2 DWR Example in Python

## Description

This is a simple example that shows the basic operation of PlanSys2. You should specify a PDDL domain for the dock-worker robot, implement actions to simulate the execution, and use the PlanSys2 Terminal to insert commands.

## How to run

In terminal 1:

```
ros2 launch week_7_solution plansys2_dwr.launch.yaml
```

In terminal 2:

```
ros2 run plansys2_terminal plansys2_terminal        # enters in PlanSys2 Terminal

(:objects r1 - robot l1 l2 - location k1 k2 - crane
            p1 q1 p2 q2 - pile ca cb cc cd ce cf pallet - container)


set instance r1 robot
set instance l1 location
set instance l2 location
set instance k1 crane
set instance k2 crane
set instance p1 pile
set instance q1 pile
set instance p2 pile
set instance q2 pile
set instance ca container
set instance cb container
set instance cc container
set instance cd container
set instance ce container
set instance cf container
set instance pt1 container
set instance pt2 container
set instance pt3 container
set instance pt4 container


get problem instances                               # Checks instances

set predicate (adjacent l1 l2)
set predicate (adjacent l2 l1)
set predicate (attached p1 l1)
set predicate (attached q1 l1)
set predicate (attached p2 l2)
set predicate (attached q2 l2)
set predicate (belong k1 l1)
set predicate (belong k2 l2)
set predicate (in ca p1)
set predicate (in cb p1)
set predicate (in cc p1)
set predicate (in cd q1)
set predicate (in ce q1)
set predicate (in cf q1)
set predicate (on ca pt1)
set predicate (on cb ca) 
set predicate (on cc cb) 
set predicate (on cd pt3) 
set predicate (on ce cd) 
set predicate (on cf ce)
set predicate (top cc p1) 
set predicate (top cf q1) 
set predicate (top pt2 p2) 
set predicate (top pt4 q2)
set predicate (at r1 l1) 
set predicate (unloaded r1) 
set predicate (occupied l1) 
set predicate (empty k1) 
set predicate (empty k2)

get problem predicates                                # Checks predicates


set goal (and (in ca p2) (in cb q2) (in cc p2) (in cd q2) (in ce q2) (in cf q2))

get plan                                              # Creates plan and shows it
run                                                   # Creates plan and runs it
```
