Dum-E mind
==========

How a request becomes a robot command
--------------------------------------

    you type  ->  local parser  ->  model  ->  resolver  ->  robot_command.json
                  (fast, exact)  (Qwen)     (angles)     (five real numbers)

The model is small (Qwen3-0.6B) and does one job only: pick an action and a
direction. It never sees or produces a number.

    {"action":"move","direction":"up-left"}

The four control words -- home, stop, status, help -- are recognised by
keyword and answered without loading the model at all, so they return
instantly. A request that also names a direction or a number is still treated
as movement ("help me turn left" is a turn). Anything the parser does not
recognise falls through to the model.

Everything numeric is decided in code, so the same request always produces the
same angles and a wrong number can never be invented.


Two stages
----------

1. The local parser (mind.py) answers without the model whenever it can:
   any direction word, any explicit joint/base number, and any gripper
   instruction. It covers about 80% of real requests.

2. Whatever is left goes to the model: home, stop, status, help, and anything
   unclear. The model loads lazily on the first such request, so a session of
   pure movement never pays for it.

A request that names a direction is never guessed at: "lower the arm to the
left" is down-left, and "raise it and turn right" is up-right.


Direction table
---------------

    request              action  base  joint1  joint2  joint3
    -------------------  ------  ----  ------  ------  ------
    turn left            turn    135   -       -       -
    turn right           turn     45   -       -       -
    tilt up              tilt      -   120    -60     -30
    tilt down            tilt      -    60    -90     -60
    move up-left         move    135   120    -60     -30
    move up-right        move     45   120    -60     -30
    move down-left       move    135    60    -90     -60
    move down-right      move     45    60    -90     -60
    turn to the middle   turn     90   -       -       -
    middle / center      turn     90   -       -       -
    up-middle            move     90   120    -60     -30
    down-middle          move     90    60    -90     -60
    home                 home     90    90    -60     -60   gripper 50
    stop / status / help -        -      -      -       -       -

A plain turn swings 45 degrees off the middle, and a diagonal turns exactly as
far as the plain turn it is named after: move up-left is not a bigger turn than
turn left.

Centering needs no verb either. "middle", "center", "centre", "straight",
"straight ahead", "halfway" and "ahead" each put the base back to 90 and leave
every other joint where it is, so "middle" on its own is a command. Only the
base moves -- a dash means the joint holds its current angle. "go ahead" is not
a command, and a middle word inside a longer sentence is left to the model:
"center of gravity" and "replace the middle gearbox" are not instructions.

"middle" is a real direction, not an alias for the default, which is why the
model is trained on it and why a turn to the middle may not also steer the arm.
Pairing it with up or down gives the two compound directions, exactly as
up-left pairs the left base with that same tilt: "up middle" and "raise the arm
to the middle" both center the base and tilt up. A middle word beside a real
left or right is ambiguous -- "turn left to the middle" names two targets -- so
that is left to the model.

Asking for the whole swing takes the base the full 90 to its mechanical stop.
These two rows are produced by the parser, not the model, so they are reachable
without naming a turn verb ("all the way left" is a command on its own):

    request                     action  direction     base
    --------------------------  ------  ------------  ----
    turn left                   turn    left           135
    turn all the way left       turn    hard-left      180
    turn right                  turn    right           45
    turn all the way right      turn    hard-right       0

The intensifiers are "all the way", "as far as you can", "fully", "completely",
"entirely", "hard", "far" and "max". Hardening only applies to a turn on its
own: "lower the arm all the way to the left" is still down-left at 135, and
"the arm is fully extended, turn left" is still left at 135.

robot_command.json
-------------------

The output the hardware reads. Every file holds all five angles as real
numbers: there is no null and no dash. A joint the command did not mention
carries the angle the arm is already holding, so the file is always the
complete post-command pose. Nulls exist only inside the parser, where they
mean "unchanged", and they are filled in before anything is written.

The first pose is the home state above, so a command that names only the base
still arrives as a full set of five angles.

Ranges: base 0..180, joint1 0..180, joint2 and joint3 -150..150, gripper
0..100.

Numbers are accepted as digits or words ("one hundred and thirty five" is
135) and always win over the table.


robot_state.json
----------------

Always five concrete numbers, never null: the pose Dum-E remembers, used to
fill in any joint a command leaves out. Only the joints a command actually
mentions are changed, so "turn left" does not disturb the arm pose. With no
file present the first pose is the home state.


Files
-----

    mind.py                     the runtime: parser, model call, resolver, state
    system_prompt.txt           the model's instructions, and the only copy
    test_resolver.py            211 assertions, no model needed
    verify_model.py             end-to-end check against the held-out split
    generate_robot_dataset.py   rebuilds robot_dataset.jsonl / _eval.jsonl
    train_robot.py              QLoRA fine-tune, 1000 steps
    merge_robot.py              merges the adapter into dum-e-qwen3-merged
    robot_command.json          the output, read by the hardware
    robot_state.json            the remembered pose


Rebuilding the model
--------------------

    python test_resolver.py
    python generate_robot_dataset.py
    python train_robot.py
    python merge_robot.py
    python verify_model.py

Edit system_prompt.txt and rerun from generate_robot_dataset.py onwards: the
prompt is embedded in every training row, so changing it without rebuilding
the dataset leaves the model and its training data disagreeing. verify_model.py
refuses to run if it detects that.

verify_model.py scores the pipeline the way the robot actually uses it: the
local parser is checked against every held-out prompt, and the model is scored
only on the prompts that really reach it. Scoring the model alone is
misleading, because most held-out prompts never get that far.

Last full run: local 1200/1200, model 118/120, end-to-end 99.8%.


Note on inference
-----------------

Qwen3 defaults to a thinking mode that emits a <think> preamble and never
reaches the JSON. Both mind.py and train_robot.py call apply_chat_template
with enable_thinking=False, and they must agree: training without it teaches a
format the runtime never asks for.
