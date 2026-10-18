# SKYFORCE

**A drone designed to safely catch another drone, using a net and a parachute.**

SKYFORCE is a design for a "drone catcher": a six-propeller drone that is guided by a ground radar to find a rogue drone (for example, one flying near an airport or a crowded event), catch it in a net, and bring it down safely and in one piece. It does not shoot, crash into, or destroy the intruder.

It started as a team entry for the **ORION UAV Design Challenge 2026** (Team 08, Senior Track, organised by P.E.G.A.S.U.S. with S.K.E.T.C.H. and CCGP, IIST). It is now being continued as a personal project and a possible future application.

> **Please read this first: this is a design, not a finished product.**
> Everything here is on paper and in computer simulation. **No physical drone has been built or flown**, and nothing has been tested in the real world. All the numbers come from calculations and assumptions, which are labelled as such in the documents.

---

## The idea in plain words

1. **Spot it.** A ground radar notices an unknown drone approaching a protected area and passes its position to a laptop (the "ground control station").
2. **Decide.** The laptop works out whether the intruder will actually enter the protected zone and whether there is enough time to react. If not, the SKYFORCE drone stays on the ground. This is a deliberate safety choice: when in doubt, do nothing.
3. **Launch and chase.** If the answer is yes, SKYFORCE takes off and flies toward the intruder. Far away it follows the radar. When close, its own camera takes over.
4. **Catch.** At roughly 5 to 15 metres, a gas-powered launcher fires a net (about 4 m wide) at the intruder.
5. **Bring it down gently.** A parachute slows the caught drone. If the catch happens somewhere unsafe (over people, for example), SKYFORCE slowly tows the intruder to the nearest pre-marked safe landing area before letting go. If the intruder is too heavy (over 5 kg), SKYFORCE does not tow it and requests the parachute instead.

## Key facts at a glance

| | |
|---|---|
| Type | Electric drone with 6 propellers (a "hexarotor") |
| Weight | 18 kg (23.5 kg while towing a 5 kg target). The competition limit was 25 kg |
| Top speed (target) | 25 m/s (90 km/h) |
| Catching tool | CO2-fired net (4 m opening) plus a 2.5 m parachute |
| Detection | Ground radar, at least 1 km range |
| Estimated cost of one drone | about INR 5.98 lakh (INR 598,037), excluding radar, laptop, spares and shipping |
| Status | Preliminary design and simulation only |

---

## What is inside this repository

| File or folder | What it is | Who it is for |
|---|---|---|
| `Final_design_report.pdf` | **Start here.** The full written report: the problem, the design, the maths behind it, the simulation results, cost, risks and safety. | Anyone who wants the whole story |
| `Final_Hackathon_PPt.docx` | The presentation made for the hackathon. | Anyone who wants a quick overview |
| `SKYFORCE_Sizing_Calculations.pdf` | Step-by-step calculations for weight, thrust, battery size, arm strength and parachute size. | Engineers checking the numbers |
| `SKYFORCE_Communications_and_Wiring_Architecture.pdf` | How the radar, laptop, radio, flight computer, camera and net trigger are connected, plus the power wiring. | Engineers and electronics people |
| `SKYFORCE_Bill_of_Equipment.xlsx` | Parts list with quantities, prices and where each price came from. | Anyone curious about cost |
| `CAD PART/` | 3D models of the drone (frame, motor mounts, propeller, landing gear, connectors and the full assembly) in `.step` format, plus one Fusion 360 file (`Top assembly.f3d`). | Anyone who wants to view or build on the physical design |
| `skyforce_intercept_simulation_final_report.m` | The MATLAB program that simulates the chase and produces all the results. | Anyone who wants to re-run the simulation |
| `SIMULATIONS PART/` | The output of that program: 11 pictures (`.png`) of the simulated scenarios and several data tables (`.csv`). | Anyone who wants to see the results without running anything |

---

## How to use this repository

### Option 1: Just read it (no software needed)

1. Open **`Final_design_report.pdf`**. The first page has a one-page summary, and the later sections go into detail.
2. To see the simulated chases, open the pictures in **`SIMULATIONS PART/`**. Each picture name describes the situation, for example:
   - `figure_01`: chasing a slower target
   - `figure_02`: intercepting a faster target by flying to where it will be
   - `figure_03`: catch happens in an unsafe spot, so the drone tows the target to safety
   - `figure_04` and `figure_05`: situations where the drone correctly decides **not** to take off
   - `figure_06` and `figure_11`: a target that is too heavy
   - `figure_07`: the target leaves the protected zone and the chase is stopped
3. Open any `.csv` file in Excel or Google Sheets to see the raw numbers. For example, `skyforce_simulation_summary.csv` is a one-line-per-scenario summary of the results.
4. The parts list is in `SKYFORCE_Bill_of_Equipment.xlsx` (opens in Excel or Google Sheets).

### Option 2: Look at the 3D model

The files in `CAD PART/` are in `.step` format, which most 3D design programs can open, such as Fusion 360, SolidWorks, FreeCAD (free) or Onshape (free, online). Start with **`Full_assembly.step`** to see the whole drone, or open the individual parts one at a time. `Top assembly.f3d` opens in Autodesk Fusion 360.

### Option 3: Re-run the simulation

You will need **MATLAB** (version R2018b or newer).

1. Download or clone this repository.
2. Open MATLAB and set the **current folder** to the folder containing `skyforce_intercept_simulation_final_report.m`. Output files are saved to the current folder.
3. In the MATLAB command window, type:
   ```matlab
   skyforce_intercept_simulation_final_report
   ```
4. Wait for it to finish. It will run 8 scenarios, draw the figures, and save the images (`.png`, plus editable `.fig` files) and the `.csv` tables in your current folder.

Notes:
- The numbers at the top of the script (drone weight, speed limits, safe-area sizes and so on) are the settings used in the report. Change them to try your own "what if" cases.
- The saved figure list (`skyforce_figure_export_manifest.csv`) contains file paths from the author's own computer. This is normal and does not affect anything.

---

## What the simulation does and does not show

**It shows** that the decision logic works in the simulated scenarios: when to launch, how to chase a slower or faster target, when to tow versus release, what to do with a too-heavy target, and when to stop a chase that would run the battery or radio link past safe limits.

**It does not show** that a real drone can do this. The simulation treats the drone as a simple moving dot. It does not model wind, a real net hitting a real drone, the parachute opening, real battery behaviour, or sensor errors. A "catch" in the simulation only means the drone got within 5 to 15 metres in the right position (the report calls it a "geometric capture candidate").

## Honest limitations and next steps

Things still to be done, taken from the report:

- Check the weight list against real, purchased parts (a few parts in the cost sheet are substitutes for the originally chosen ones).
- Choose an actual battery, radar, radio and camera.
- Replace assumed values (air drag, propeller thrust at speed) with real data or test results.
- Run one continuous start-to-landing energy calculation.
- Insert the final CAD figures into the report.
- Build and test, step by step, starting with small bench tests.

## Safety, law and ethics

- The design is for **lawful use by authorised operators only**. Rules for flying drones, using radio frequencies, and stopping other people's drones differ by country. In India these must be checked before any real use. Nothing here is legal advice or approval for operation.
- The net trigger is designed to need a deliberate operator command plus a physical arming switch, and to stop working if the radio link is lost.
- The design is meant to catch without harming people or property. It does not promise a 100% catch rate, and every safety rule is built to fail toward "do nothing".

## Team

Yuvraj Singh Tomar (Team Leader), Dipanshu Kumar Kushwaha (CAD / 3D modelling), Shaurya Singh, Aditya Kaundilya.

## Source and licence

The repository is at <https://github.com/Yuvraj2412/SKYFORCE>. No open-source licence has been chosen yet, so by default all rights are reserved. If you would like to reuse anything here, please contact the repository owner.
