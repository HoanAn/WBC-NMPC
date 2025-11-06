import re
import numpy as np
import matplotlib.pyplot as plt

# Path to your file
filename = "build/State.txt"

sections = [
    "q measurement",
    "q0 MPC last solution",
    "Predicted q MPC over horizon",
    "v measurement"
]

# Read the text file
with open(filename, "r") as f:
    text = f.read()

# Split into blocks separated by lines of dashes
blocks = re.split(r"-{5,}", text)

# Function to extract numeric arrays for a given section
def extract_section_data(section_name):
    data = []
    for block in blocks:
        if section_name in block:
            # Get the numeric text following the section name
            matches = re.findall(rf"{section_name}:\s*([\s\S]*?)(?=\n\S|\Z)", block)
            for m in matches:
                try:
                    nums = [float(x) for x in m.split()]
                    data.append(nums)
                except ValueError:
                    continue
    return np.array(data)

# Iterate over each section and make 3 subplots
for section_name in sections:
    data = extract_section_data(section_name)
    if data.size == 0:
        print(f"⚠️ No data found for '{section_name}'")
        continue

    # Use only the first 3 components
    components = data[:, :3]

    # Create a figure with 3 subplots (one per component)
    fig, axes = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
    fig.suptitle(f"{section_name} — First 3 Components", fontsize=14)

    for i in range(3):
        axes[i].plot(components[:, i], label=f"Component {i+1}")
        axes[i].set_ylabel(f"Comp {i+1}")
        axes[i].grid(True)
        axes[i].legend(loc="best")

    axes[-1].set_xlabel("Sample index")

    plt.tight_layout(rect=[0, 0, 1, 0.96])  # Leave space for the main title
    plt.show()