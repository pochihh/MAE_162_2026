"""
User-defined mission tasks. Set MODE below
"""

from robot.fsm_helpers import course_parameters as cp
from robot.fsm_helpers.course_parameters import SQ

MODE = "FINAL"
# MODE = "helpers"
# MODE = "RANDOM"

if MODE == "RANDOM":
    tasks = [{
        "state": "NAV",
        "path_planner": "pp",
        "waypoints": [(0, 10*24*25.4)],
    },
    ]

elif MODE == "vel_profile_lapf":
    # Notes: may need to tune WP_SPACING_LAPF, TOLERANCE_MM_LAPF
    tasks = [
        {
            "state": "NAV",
            "path_planner": "lapf",
            "waypoints": [(0*SQ, 0*SQ), (0*SQ, 5*SQ), (0.75*SQ, 5*SQ)],
            "goal_heading": 0
        }
    ]

elif MODE == "feedforward":
    tasks = [
        {
            "state": "NAV",
            "path_planner": "pp",
            "waypoints": [(0*SQ, 10*SQ)],
            # "goal_heading": 0
        }
    ]

# =============================================================================
# Navigation
# =============================================================================

elif MODE == "TUNE_TURN":
    tasks = [{
        "state": "NAV",
        "path_planner": "pp",
        #"waypoints": [(0*SQ, 6*SQ)],
        #"waypoints": [(0*SQ, 1*SQ), (1*SQ, 1*SQ)],
        "goal_heading": 0.0
    }]

elif MODE == "SQUARE2":
    tasks = [
        {"state": "NAV", "path_planner": "pp", "waypoints": [
            (0*SQ, 0*SQ),
            (0*SQ, 2*SQ),
            (2*SQ, 2*SQ),
            (2*SQ, 0*SQ),
            (0*SQ, 0*SQ),
            (0*SQ, 2*SQ),
            (2*SQ, 2*SQ),
            (2*SQ, 0*SQ),
            (0*SQ, 0*SQ)
            ]
        }
    ]

elif MODE == "ALL_WAYPOINTS":
    tasks = [
        {"state": "NAV", "path_planner": "pp", "waypoints": [cp.WP1, cp.WP2, cp.WP3, cp.WP4], "goal_heading": 90.0},
        {"state": "NAV", "path_planner": "lapf", "waypoints": [cp.WP4B, cp.WP_CUSTOMER_ID], "goal_heading": 0.0},
        {"state": "NAV", "path_planner": "pp", "waypoints": [cp.WP5, cp.WP6]}
    ]

# =============================================================================
# Other single tasks
# =============================================================================

elif MODE == "IDENT":
    tasks = [{"state": "IDENT"}]

elif MODE == "WAIT":
    tasks = [{"state": "WAIT", "trigger": "green_light"}]

elif MODE == "TUNE_LAPF":
    tasks = [{
        "state": "NAV",
        "path_planner": "lapf",
        "waypoints": [(0*SQ, 3*SQ)],
        "goal_heading": 0.0
    }
    ]

elif MODE == "helpers":
    tasks = [
        {"state": "HOME"},
        {"state": "MANIP", "command": "pick", "ingredient": "bun"},
        {"state": "MANIP", "command": "pick", "ingredient": "patty"},
        {"state": "MANIP", "command": "pick", "ingredient": "bun"},
        {"state": "MANIP", "command": "place"}
    ]

# =============================================================================
# Multiple tasks
# =============================================================================

elif MODE == "FINAL":
    tasks = [
        {
            "state": "HOME"
        },
        {
            "state": "WAIT", "trigger": "green_light"
        },
        {
            "state": "NAV",
            "path_planner": "pp",
            "vision": "burger_bun",
            "waypoints": cp.WP_BURGER_BUN_1, 
            "goal_heading": 170.0
        },
        {
            "state": "MANIP",
            "command": "pick",
            "ingredient": "bun"
        },
        {
            "state": "NAV",
            "path_planner": "pp",
            "vision": "burger_patty",
            "waypoints": cp.WP_BURGER_PATTY,
            "goal_heading": 180.0
        },
        {
            "state": "MANIP",
            "command": "pick",
            "ingredient": "patty"
        },
        {
            "state": "NAV",
            "path_planner": "pp",
            "vision": "burger_bun",
            "waypoints": cp.WP_BURGER_BUN_2,
            "goal_heading": 180.0
        },
        {
            "state": "MANIP", 
            "command": "pick", 
            "ingredient": "bun"
        },
        {
            "state": "NAV", 
            "path_planner": "pp", 
            "waypoints": [cp.WP1, cp.WP2, cp.WP3, cp.WP4]
        },
        {
            "state": "NAV",
            "path_planner": "lapf",
            "vision": "face",
            "waypoints": [cp.WP4B, cp.WP_CUSTOMER_ID],
            "goal_heading": 0.0
        },
        {
            "state": "IDENT"
        },
        {
            "state": "NAV", 
            "path_planner": "pp",
            "waypoints": [cp.WP5],
            "goal_heading": 0.0
        },
        {
            "state": "MANIP", "command": "place"
        }
    ]

elif MODE == "LAPF_TO_IDENT":
    tasks = [{
        "state": "NAV",
        "path_planner": "lapf",
        "waypoints": [(0.75*SQ, 5*SQ)],
        "goal_heading": 0.0
    },
    {
        "state": "IDENT",
    }
    ]

elif MODE == "TEST_NAV":
    tasks = [
        {"state": "NAV", "path_planner": "pp", "vision": "burger_bun", "waypoints": cp.WP_BURGER_BUN_1},
        {"state": "NAV", "path_planner": "pp", "vision": "burger_patty", "waypoints": cp.WP_BURGER_PATTY},
        {"state": "NAV", "path_planner": "pp", "vision": "burger_bun", "waypoints": cp.WP_BURGER_BUN_2},
        {"state": "NAV", "path_planner": "pp", "waypoints": [cp.WP1, cp.WP2, cp.WP3, cp.WP4]},
        {"state": "NAV", "path_planner": "lapf", "vision": "face", "waypoints": cp.WP_CUSTOMER_ID},
        {"state": "NAV", "path_planner": "pp", "vision": "customer", "waypoints": [cp.WP5, cp.WP_CUSTOMER_1]},
        {"state": "NAV", "path_planner": "pp", "vision": "stop_sign", "waypoints": cp.WP_STOP_SIGN},
        {"state": "NAV", "path_planner": "pp", "waypoints": cp.WP6}
    ]