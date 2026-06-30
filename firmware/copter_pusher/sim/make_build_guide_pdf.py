#!/usr/bin/env python3
"""Generate a complete build guide PDF for the MetroAir pusher firmware."""
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.lib import colors
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.platypus import (SimpleDocTemplate, Paragraph, Spacer, Preformatted,
                                PageBreak, Table, TableStyle, HRFlowable, ListFlowable, ListItem)
from reportlab.lib.enums import TA_LEFT
import os

REPO = "/home/user/PARAGLIDER-RTG/firmware/copter_pusher"
OUT  = "/tmp/MetroAir_Pusher_Firmware_Build_Guide.pdf"

def rd(p):
    with open(os.path.join(REPO, p)) as f:
        return f.read().rstrip("\n")

ss = getSampleStyleSheet()
H1 = ParagraphStyle('H1', parent=ss['Heading1'], fontSize=17, spaceBefore=10, spaceAfter=8,
                    textColor=colors.HexColor('#0b3d5c'))
H2 = ParagraphStyle('H2', parent=ss['Heading2'], fontSize=13, spaceBefore=10, spaceAfter=5,
                    textColor=colors.HexColor('#13628f'))
H3 = ParagraphStyle('H3', parent=ss['Heading3'], fontSize=11, spaceBefore=8, spaceAfter=3,
                    textColor=colors.HexColor('#1b6b3a'))
BODY = ParagraphStyle('BODY', parent=ss['BodyText'], fontSize=9.6, leading=13.5, alignment=TA_LEFT)
NOTE = ParagraphStyle('NOTE', parent=BODY, leftIndent=8, textColor=colors.HexColor('#7a2c00'),
                      backColor=colors.HexColor('#fff4e6'), borderPadding=5, spaceBefore=4, spaceAfter=6)
WARN = ParagraphStyle('WARN', parent=BODY, leftIndent=8, textColor=colors.HexColor('#7a0010'),
                      backColor=colors.HexColor('#ffe9ec'), borderPadding=5, spaceBefore=4, spaceAfter=6)
CODE = ParagraphStyle('CODE', parent=ss['Code'], fontSize=7.7, leading=9.6,
                      backColor=colors.HexColor('#f3f4f6'), borderPadding=5,
                      textColor=colors.HexColor('#111111'), spaceBefore=4, spaceAfter=6)
TITLE = ParagraphStyle('TITLE', parent=ss['Title'], fontSize=24, textColor=colors.HexColor('#0b3d5c'))
SUB = ParagraphStyle('SUB', parent=ss['Normal'], fontSize=11, textColor=colors.HexColor('#444'),
                     spaceBefore=6)

E = []
def P(t, s=BODY): E.append(Paragraph(t, s))
def code(t): E.append(Preformatted(t, CODE))
def gap(h=4): E.append(Spacer(1, h))
def hr(): E.append(HRFlowable(width="100%", color=colors.HexColor('#cccccc'), spaceBefore=6, spaceAfter=6))

# ---------------- COVER ----------------
gap(60)
P("MetroAir Hexa + Pusher", TITLE)
P("ArduCopter Firmware Build Guide", ParagraphStyle('x', parent=TITLE, fontSize=16, textColor=colors.HexColor('#13628f')))
gap(10)
P("Pusher-propeller torque feed-forward compensation", SUB)
P("Board: MicoAir743v2 &nbsp;•&nbsp; Firmware: ArduCopter 4.6.x &nbsp;•&nbsp; Frame: Hexa-X + EDF pusher", SUB)
gap(20)
P("This document is fully self-contained: it includes the source patch, parameters, "
  "and step-by-step commands to apply, build (SITL and the real <b>.apj</b>/bootloader), "
  "flash, test, and calibrate the firmware on a fresh machine. Hand it to a new chat or "
  "another engineer and they can build it end-to-end.", BODY)
gap(14)
tbl = Table([
    ["Airframe", "262 mm hexa (131 mm adjacent), MTOW 600 g+"],
    ["Lift motors", "F1404 KV3800/4600, 3.5\" x 2\" props (6x)"],
    ["Pusher", "QF1611 KV6000 / 30 mm EDF on body +X"],
    ["Battery", "LiPo 1300 mAh 4S1P"],
    ["FCU", "MicoAir743v2 (ArduCopter 4.6.x)"],
    ["Pusher output", "SERVO7_FUNCTION = 58 (RCIN8 passthrough)"],
], colWidths=[90, 360])
tbl.setStyle(TableStyle([
    ('FONT',(0,0),(-1,-1),'Helvetica',8.5),
    ('FONT',(0,0),(0,-1),'Helvetica-Bold',8.5),
    ('TEXTCOLOR',(0,0),(0,-1),colors.HexColor('#0b3d5c')),
    ('BACKGROUND',(0,0),(0,-1),colors.HexColor('#eef3f7')),
    ('GRID',(0,0),(-1,-1),0.4,colors.HexColor('#cccccc')),
    ('VALIGN',(0,0),(-1,-1),'MIDDLE'),
    ('LEFTPADDING',(0,0),(-1,-1),6),('TOPPADDING',(0,0),(-1,-1),3),('BOTTOMPADDING',(0,0),(-1,-1),3),
]))
E.append(tbl)
E.append(PageBreak())

# ---------------- 0. WHY ----------------
P("0. Why this firmware exists", H1)
P("Across four flight logs the airframe repeatedly pitched <b>nose-down on its own</b> when "
  "the pusher spun up, and crashed. In log <font face='Courier'>00000170.BIN</font> the attitude "
  "controller commanded the <b>maximum +75&deg;/s nose-up</b> recovery and <b>saturated all six "
  "lift motors (~1950&micro;s)</b>, yet the vehicle still pitched to <b>-84&deg;</b> and dove from 22 m. "
  "Pilot input was near neutral the whole time.", BODY)
P("<b>Cause:</b> the pusher's thrust line is offset from the centre of gravity, so its thrust "
  "produces a <b>moment</b> the stock multicopter controller has no model of. It only reacts "
  "<i>after</i> an attitude error appears &mdash; too late at high pusher thrust.", BODY)
P("<b>Fix:</b> feed-forward the pusher's disturbance moment so the main rotors cancel it "
  "<i>before</i> an error develops. This guide builds an ArduCopter firmware that does exactly "
  "that, gated behind parameters and off by default.", BODY)

P("0.1 The model (torque equations)", H2)
P("A body +X pusher produces force and moment (T_p, Q_p &prop; &Omega;&sup2; &prop; pusher&sup2;):", BODY)
code("F_p   = [ T_p , 0 , 0 ]                        forward thrust\n"
     "tau_p = [ sigma*Q_p , z_p*T_p , -h*T_p ]       roll , pitch , yaw\n"
     "        \\________/   \\______/    \\_____/\n"
     "        motor         vertical     Y-offset of\n"
     "        reaction      offset of    thrust line (h)\n"
     "        torque        thrust (z_p)")
P("The main rotors must generate <b>tau_main = tau_des &minus; tau_p</b>, i.e. add <b>&minus;tau_p</b> "
  "as feed-forward. The vertical-offset <b>pitch</b> term (z_p&middot;T_p) is the one that crashed the "
  "aircraft; it is the primary gain to calibrate.", BODY)
hr()

# ---------------- 1. PREREQS ----------------
P("1. Prerequisites (one-time toolchain setup)", H1)
P("ArduPilot builds on macOS and Linux. Pick your OS.", BODY)
P("1.1 macOS", H2)
code("# Xcode command-line tools + Homebrew\n"
     "xcode-select --install\n"
     "/bin/bash -c \"$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)\"\n\n"
     "# Python + git\n"
     "brew install python git gawk\n\n"
     "# (for building the real board firmware) ARM cross compiler\n"
     "brew install --cask gcc-arm-embedded   # provides arm-none-eabi-gcc")
P("1.2 Ubuntu / Debian Linux", H2)
code("sudo apt update\n"
     "sudo apt install -y git python3 python3-pip gawk gcc-arm-none-eabi")
P("1.3 ArduPilot Python dependencies (both OSes)", H2)
code("python3 -m pip install --user pymavlink MAVProxy empy==3.3.4 pexpect future")
hr()

# ---------------- 2. SOURCE ----------------
P("2. Get the ArduPilot source", H1)
P("Match the branch to the version running on the board (4.6 here). The <font face='Courier'>"
  "--recurse-submodules</font> flag is required.", BODY)
code("git clone --recurse-submodules -b Copter-4.6 \\\n"
     "    https://github.com/ArduPilot/ardupilot.git\n"
     "cd ardupilot\n"
     "# if you forgot the submodules:\n"
     "git submodule update --init --recursive")
P("Also clone (or copy) the project repo that holds the patch and params:", BODY)
code("git clone <your PARAGLIDER-RTG repo url>\n"
     "# the firmware package lives in:\n"
     "#   PARAGLIDER-RTG/firmware/copter_pusher/")
hr()

# ---------------- 3. APPLY PATCH ----------------
P("3. Apply the pusher feed-forward patch", H1)
P("From inside the <font face='Courier'>ardupilot</font> checkout:", BODY)
code("git apply /path/to/PARAGLIDER-RTG/firmware/copter_pusher/ardupilot_patch/pusher_ff.patch\n\n"
     "# verify it went in:\n"
     "grep -c \"Pusher (X-axis propulsion) torque feed-forward\" \\\n"
     "    libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp   # -> 1")
P("If <font face='Courier'>git apply</font> reports rejects (line numbers drift between point "
  "releases), apply the four edits by hand &mdash; only two files change. The exact blocks are in "
  "Appendix A of this document.", NOTE)
hr()

# ---------------- 4. SITL ----------------
P("4. Build &amp; test in SITL first (no hardware risk)", H1)
P("SITL's stock hexa has no pusher, so it cannot reproduce the aerodynamic disturbance &mdash; but it "
  "<b>does</b> confirm the patch is wired correctly: with the compensation ON, sweeping the pusher RC "
  "channel injects a pitch command proportional to pusher&sup2;.", BODY)
code("./waf configure --board sitl\n"
     "./waf copter\n\n"
     "# launch a hexa SITL\n"
     "Tools/autotest/sim_vehicle.py -v ArduCopter -f hexa --console\n\n"
     "# in a second terminal, run the probe:\n"
     "python3 /path/to/PARAGLIDER-RTG/firmware/copter_pusher/sim/sitl_pusher_probe.py \\\n"
     "        --conn udp:127.0.0.1:14550 --gain 0.12")
P("Or run the whole thing with one wrapper:", BODY)
code("/path/to/PARAGLIDER-RTG/firmware/copter_pusher/sim/run_sitl_mac.sh \\\n"
     "    /path/to/ardupilot  /path/to/PARAGLIDER-RTG")
P("<b>PASS criteria:</b> the EN=0 column stays ~flat as pusher PWM rises; the EN=1 column shows pitch "
  "growing with pusher (sign follows ATC_PUSH_PIT). That proves the term is live and reads the right "
  "channel before you ever fly it.", BODY)
hr()

# ---------------- 5. BUILD REAL FW ----------------
P("5. Build the real firmware (.apj + bootloader)", H1)
P("For the MicoAir743v2 flight controller:", BODY)
code("./waf configure --board MicoAir743v2\n"
     "./waf copter\n\n"
     "# flashable firmware image:\n"
     "#   build/MicoAir743v2/bin/arducopter.apj\n"
     "#   build/MicoAir743v2/bin/arducopter      (elf, for debugging)\n\n"
     "# bootloader (only if you must (re)flash the bootloader):\n"
     "./waf configure --board MicoAir743v2 --bootloader\n"
     "./waf bootloader\n"
     "#   build/MicoAir743v2/bootloader/MicoAir743v2_bl.bin")
P("The <font face='Courier'>.apj</font> is what you give your friend / load in Mission Planner. The "
  "bootloader rarely needs reflashing &mdash; only do it if instructed.", NOTE)
hr()

# ---------------- 6. FLASH ----------------
P("6. Flash the firmware", H1)
P("6.1 Mission Planner (recommended)", H2)
P("Setup &rarr; Install Firmware &rarr; <b>Load custom firmware</b> &rarr; pick "
  "<font face='Courier'>arducopter.apj</font>. Connect the board over USB in DFU/normal mode as "
  "prompted.", BODY)
P("6.2 Command line (uploader)", H2)
code("python3 ardupilot/Tools/scripts/uploader.py \\\n"
     "    build/MicoAir743v2/bin/arducopter.apj")
hr()

# ---------------- 7. CONFIG ----------------
P("7. Load the vehicle configuration", H1)
P("Load <font face='Courier'>params/metroair_pusher.param</font> (Mission Planner &rarr; Config &rarr; "
  "Full Parameter List &rarr; Load from file &rarr; Write). It also moves the flight-mode switch "
  "<b>off RC8 to RC7</b> and enables the <b>battery failsafes</b> &mdash; both contributed to the crashes. "
  "The full file is in Appendix B.", BODY)
hr()

# ---------------- 8. CALIBRATION ----------------
P("8. Calibration procedure (safety critical)", H1)
P("The compensation term is new and untested on your airframe. Bring it up incrementally; never "
  "start with a guessed non-zero gain.", WARN)
steps = [
 "<b>Mechanical first.</b> The best fix is to put the EDF thrust line through the CG height. The "
 "feed-forward handles the <i>residual</i> offset. Until the pitch gain is calibrated, do NOT run the "
 "pusher above ~50%.",
 "<b>Flight 0 &mdash; prove it's stock.</b> With ATC_PUSH_EN=0, fly a normal ALT_HOLD hover. Behaviour "
 "must be unchanged. (Move the mode switch to RC7 first.)",
 "<b>Measure the disturbance.</b> Put lua/pusher_calib.lua in APM/scripts/, set SCR_ENABLE=1. Fly "
 "ALT_HOLD, keep the pitch stick neutral, step the pusher 10-20-30-40-50%. After landing read the "
 "PUSH-CAL table on the GCS: mean pitch per pusher band. Negative pitch at higher pusher = nose-down "
 "disturbance.",
 "<b>Tune ATC_PUSH_PIT.</b> Set ATC_PUSH_EN=1, start ATC_PUSH_PIT=0.02. Repeat the stepped test. If "
 "the nose still drops, increase 0.01-0.02 at a time; if it now pitches up, you overshot (or flip the "
 "sign). Goal: mean pitch ~0 as pusher rises with pitch stick neutral.",
 "<b>Trim roll/yaw.</b> With pitch handled, null residual roll (ATC_PUSH_RLL) and yaw drift "
 "(ATC_PUSH_YAW) the same way.",
 "<b>Sanity limit.</b> If any |gain| must exceed ~0.3, the mechanical offset is too large &mdash; fix "
 "the mount, do not mask it in software (the simulation shows even feed-forward saturates beyond the "
 "rotors' authority).",
]
E.append(ListFlowable([ListItem(Paragraph(s, BODY), leftIndent=6) for s in steps],
                      bulletType='1', bulletFontSize=9))
hr()

# ---------------- 9. PARAM REFERENCE ----------------
P("9. New parameter reference", H1)
prm = Table([
    ["Parameter","Default","Meaning"],
    ["ATC_PUSH_EN","0","Master enable (0 = identical to stock)"],
    ["ATC_PUSH_RC","8","RC input channel that drives the pusher"],
    ["ATC_PUSH_PIT","0","Pitch FF gain — positive commands nose-up (the crash-fix term)"],
    ["ATC_PUSH_RLL","0","Roll FF gain — cancels pusher motor reaction torque"],
    ["ATC_PUSH_YAW","0","Yaw FF gain — cancels Y-offset yaw moment"],
], colWidths=[80,45,325])
prm.setStyle(TableStyle([
    ('FONT',(0,0),(-1,-1),'Helvetica',8.3),
    ('FONT',(0,0),(-1,0),'Helvetica-Bold',8.6),
    ('FONT',(0,1),(0,-1),'Courier',8),
    ('BACKGROUND',(0,0),(-1,0),colors.HexColor('#0b3d5c')),
    ('TEXTCOLOR',(0,0),(-1,0),colors.white),
    ('GRID',(0,0),(-1,-1),0.4,colors.HexColor('#cccccc')),
    ('ROWBACKGROUNDS',(0,1),(-1,-1),[colors.white,colors.HexColor('#f2f6f9')]),
    ('VALIGN',(0,0),(-1,-1),'MIDDLE'),
    ('LEFTPADDING',(0,0),(-1,-1),5),('TOPPADDING',(0,0),(-1,-1),3),('BOTTOMPADDING',(0,0),(-1,-1),3),
]))
E.append(prm)
gap(6)
P("Implementation: injected in <font face='Courier'>AC_AttitudeControl_Multi::rate_controller_run()</font> "
  "right after the rate PIDs:", BODY)
code("p2 = pusher_norm^2                              // ~ T_p, Q_p  (command^2)\n"
     "set_roll ( get_roll()  + ATC_PUSH_RLL * p2 )   // cancels  sigma*Q_p\n"
     "set_pitch( get_pitch() + ATC_PUSH_PIT * p2 )   // cancels  z_p*T_p   (crash term)\n"
     "set_yaw  ( get_yaw()   + ATC_PUSH_YAW * p2 )   // cancels  -h*T_p")
E.append(PageBreak())

# ---------------- APPENDIX A: PATCH ----------------
P("Appendix A — Source patch (hand-apply reference)", H1)
P("Two files in <font face='Courier'>libraries/AC_AttitudeControl/</font>. If the .patch rejects, make "
  "these edits manually. Indices 30-34 must be unique in the var_info table (renumber if taken).", BODY)
P("A.1 pusher_ff.patch", H3)
code(rd("ardupilot_patch/pusher_ff.patch"))
E.append(PageBreak())
P("A.2 MANUAL_APPLY.md (exact edit blocks)", H3)
code(rd("ardupilot_patch/MANUAL_APPLY.md"))
E.append(PageBreak())

# ---------------- APPENDIX B: PARAMS ----------------
P("Appendix B — metroair_pusher.param", H1)
code(rd("params/metroair_pusher.param"))
E.append(PageBreak())

# ---------------- APPENDIX C: LUA ----------------
P("Appendix C — pusher_calib.lua (calibration logger)", H1)
P("Place in the autopilot's APM/scripts/ folder, set SCR_ENABLE=1. Reports the pusher-induced "
  "pitch/roll/yaw per pusher band so you can choose the ATC_PUSH_* gains.", BODY)
code(rd("lua/pusher_calib.lua"))
E.append(PageBreak())

# ---------------- APPENDIX D: TROUBLESHOOTING ----------------
P("Appendix D — Troubleshooting", H1)
ts = [
 ("git apply fails","Use Appendix A.2 manual edits (2 files, 4 edits). Confirm you are on Copter-4.6."),
 ("ATC_PUSH_* not visible after flash","The patch didn't compile in. Re-check the grep in Section 3, rebuild, reflash. Refresh params in Mission Planner."),
 ("waf configure: board not found","Update the source (newer ArduPilot) or check exact board name: ./waf list_boards | grep -i micoair"),
 ("submodule errors during build","git submodule update --init --recursive"),
 ("arm-none-eabi-gcc not found","Install the ARM toolchain (Section 1). SITL build does not need it; the board build does."),
 ("SITL probe shows no EN=1 effect","Confirm ATC_PUSH_EN=1, ATC_PUSH_RC=8, ATC_PUSH_PIT non-zero, and that you rebuilt after patching."),
 ("Nose still drops in flight at high pusher","Disturbance exceeds pitch authority: reduce pusher, then fix the EDF thrust-line height (mechanical) before raising the gain further."),
]
tt = Table([["Symptom","Action"]]+[[a,b] for a,b in ts], colWidths=[150,300])
tt.setStyle(TableStyle([
    ('FONT',(0,0),(-1,-1),'Helvetica',8.2),
    ('FONT',(0,0),(-1,0),'Helvetica-Bold',8.6),
    ('BACKGROUND',(0,0),(-1,0),colors.HexColor('#0b3d5c')),
    ('TEXTCOLOR',(0,0),(-1,0),colors.white),
    ('GRID',(0,0),(-1,-1),0.4,colors.HexColor('#cccccc')),
    ('ROWBACKGROUNDS',(0,1),(-1,-1),[colors.white,colors.HexColor('#f2f6f9')]),
    ('VALIGN',(0,0),(-1,-1),'TOP'),
    ('LEFTPADDING',(0,0),(-1,-1),5),('TOPPADDING',(0,0),(-1,-1),3),('BOTTOMPADDING',(0,0),(-1,-1),3),
]))
E.append(tt)
gap(10)
P("Build artifacts summary", H2)
code("SITL binary   : build/sitl/bin/arducopter\n"
     "Board firmware: build/MicoAir743v2/bin/arducopter.apj   <-- flash this\n"
     "Bootloader    : build/MicoAir743v2/bootloader/MicoAir743v2_bl.bin")

def footer(canvas, doc):
    canvas.saveState()
    canvas.setFont('Helvetica', 7.5)
    canvas.setFillColor(colors.HexColor('#888888'))
    canvas.drawString(20*mm, 12*mm, "MetroAir Hexa+Pusher — ArduCopter Firmware Build Guide")
    canvas.drawRightString(190*mm, 12*mm, "Page %d" % doc.page)
    canvas.restoreState()

doc = SimpleDocTemplate(OUT, pagesize=A4, leftMargin=20*mm, rightMargin=20*mm,
                        topMargin=16*mm, bottomMargin=18*mm,
                        title="MetroAir Pusher Firmware Build Guide")
doc.build(E, onFirstPage=footer, onLaterPages=footer)
print("wrote", OUT)
