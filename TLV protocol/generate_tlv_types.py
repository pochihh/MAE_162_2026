#!/usr/bin/env python3
"""
Generate TLV_TypeDefs.h and TLV_TypeDefs.py from TLV_TypeDefs.json
This script reads the JSON file and generates both C++ header and Python module.
"""

import json
import sys
from pathlib import Path

def generate_c_header(json_data, output_path):
    """Generate C++ header file from JSON data"""
    types = json_data.get('types', {})
    bitmasks = json_data.get('bitmasks', {})
    
    header_content = """#pragma once

#include <cstdint>

// Define TLV type constants here so both server and client can use them:
// A valid TLV type is a non-negative integer.
// This file is auto-generated from TLV_TypeDefs.json - DO NOT EDIT MANUALLY

// ============================================================================
// TLV Type Constants
// ============================================================================

"""
    
    # Sort by value for consistent output
    sorted_types = sorted(types.items(), key=lambda x: x[1])
    
    for name, value in sorted_types:
        header_content += f"constexpr uint32_t {name} = {value}U;\n"
    
    header_content += "\n"
    
    # Write to file
    with open(output_path, 'w') as f:
        f.write(header_content)
    
    print(f"Generated: {output_path}")

def generate_python_module(json_data, output_path):
    """Generate Python module from JSON data"""
    types = json_data.get('types', {})
    bitmasks = json_data.get('bitmasks', {})
    
    python_content = """\"\"\"
TLV Type Definitions - Auto-generated from TLV_TypeDefs.json
This file is auto-generated - DO NOT EDIT MANUALLY
\"\"\"

"""
    
    # ============================================================================
    # TLV Type Constants
    # ============================================================================
    python_content += "# ============================================================================\n"
    python_content += "# TLV Type Constants\n"
    python_content += "# ============================================================================\n\n"
    
    # Sort by value for consistent output
    sorted_types = sorted(types.items(), key=lambda x: x[1])
    
    # Generate constants
    for name, value in sorted_types:
        python_content += f"{name} = {value}\n"
    
    python_content += "\n# Dictionary for programmatic access\n"
    python_content += "TLV_TYPES = {\n"
    for name, value in sorted_types:
        python_content += f"    '{name}': {value},\n"
    python_content += "}\n\n"
    
    # Write to file
    with open(output_path, 'w') as f:
        f.write(python_content)
    
    print(f"Generated: {output_path}")

def main():
    script_dir = Path(__file__).parent
    json_file = script_dir / "TLV_TypeDefs.json"
    header_file = script_dir / ".." / "firmware" / "arduino" /"src" / "TLV_TypeDefs.h"
    python_file = script_dir / ".." / "ros2_ws" / "src" / "TLV_TypeDefs.py"
    
    # Read JSON file
    if not json_file.exists():
        print(f"Error: {json_file} not found", file=sys.stderr)
        sys.exit(1)
    
    with open(json_file, 'r') as f:
        data = json.load(f)
    
    # Generate C++ header
    generate_c_header(data, header_file)
    
    # Generate Python module
    python_file.parent.mkdir(parents=True, exist_ok=True)
    generate_python_module(data, python_file)
    
    print("Successfully generated TLV type definition files")

if __name__ == "__main__":
    main()

