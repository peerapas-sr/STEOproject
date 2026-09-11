---
name: toyota-misra-c
description: >-
  Enforce and verify the 22 Toyota Embedded MISRA-C coding rules.
  Use this skill whenever writing, modifying, reviewing, or refactoring C code for embedded systems,
  or when auditing code compliance against the 22 check items.
---

# Toyota Embedded MISRA-C Coding Standard (22 Rules)

This skill provides comprehensive rules, non-compliant vs. compliant examples, and an audit checklist for the 22 Toyota Embedded MISRA-C coding standard rules.

---

## 1. Quick Reference Checklist (22 Rules)

| No. | Class | Rule Description | Key Requirement |
|:---:|:---|:---|:---|
| **1** | General | `//` comment style must not be used | Use `/* ... */` comment style only |
| **2** | General | `{` and `}` should not be on the same statement line | Braces must be on separate lines (exception: `} else {`) |
| **3** | General | It should have one space between operand and operator | `a = (b + c) / d;` not `a=(b+c)/d;` |
| **4** | General | Use one assignment per one line | Split multiple assignments onto individual lines |
| **5** | General | Hard-coded "magic" integer constants should be best avoided | Use `#define` or `const` identifiers |
| **6** | General | Ternary operator should be best avoided | Replace `? :` with `if (...) { ... } else { ... }` |
| **7** | Unused Code | A project shall not contain unreachable code | Remove dead code following `return`, `goto`, etc. |
| **8** | Unused Code | A project should not contain unused object definitions | Remove variables/functions that are never referenced |
| **9** | Literals & Constants | Octal constants shall not be used | Do not prefix integer literals with `0` (e.g. `012`) |
| **10** | Literals & Constants | A `u` or `U` suffix shall be applied to all unsigned integer constants | e.g. `32768U`, `0x8000U`, `100U` |
| **11** | Declarations & Definitions | Function types shall be in prototype form with named parameters | Use `void func(uint32_t param_name);` or `(void)` if 0 params |
| **12** | Initialization | Automatic storage duration objects shall not be read before being set | Initialize local variables before reading their value |
| **13** | Essential Type Model | Operands shall not be of an inappropriate essential type | No boolean logic on integers, no negative bit-shift magnitudes |
| **14** | Expressions | Precedence of operators within expressions should be made explicit | Enclose sub-expressions in explicit parentheses `((a * b) + c)` |
| **15** | Side Effects | Floating-point expressions shall not be tested for equality or inequality | Use tolerance comparison `fabs(a - b) < TOLERANCE` |
| **16** | Control Expressions | A loop counter shall not have essentially floating type | Use integer counters for `for` and `while` loops |
| **17** | Control Expressions | Controlling expressions of `if` and iteration statements shall have essentially Boolean type | Use explicit comparison e.g. `if (x != 0)` or `if (x == 10)` |
| **18** | Control Flow | Body of iteration or selection statements shall be a compound-statement | Always wrap `if`, `else`, `while`, `for` bodies in `{ ... }` |
| **19** | Control Flow | All `if ... else if` constructs shall be terminated with an `else` statement | Final `else` is mandatory (use `/* No action */` if empty) |
| **20** | Switch Statements | An unconditional `break` statement shall terminate every switch-clause | Every non-empty `case` and `default` must end with unconditional `break` |
| **21** | Switch Statements | Every `switch` statement shall have a `default` label | `default:` is mandatory in all switch blocks |
| **22** | Switch Statements | A `default` label shall appear as either the first or the last switch label | Place `default:` at the very beginning or the very end |

---

## 2. Standard Type & Variable Prefixes (Hungarian Reference)

The Toyota embedded guideline utilizes standardized type prefix conventions:

| Prefix | Type | Example |
|:---|:---|:---|
| `u1t_` | `uint8_t` (Unsigned 8-bit) | `uint8_t u1t_data = 0U;` |
| `u2t_` | `uint16_t` (Unsigned 16-bit) | `uint16_t u2t_counter = 100U;` |
| `u4t_` | `uint32_t` (Unsigned 32-bit) | `uint32_t u4t_timestamp = 0UL;` |
| `s1t_` | `int8_t` (Signed 8-bit) | `int8_t s1t_offset = -5;` |
| `s2t_` | `int16_t` (Signed 16-bit) | `int16_t s2t_temperature = -20;` |
| `s4t_` | `int32_t` (Signed 32-bit) | `int32_t s4t_result = 0;` |
| `f4t_` | `float` (32-bit single precision) | `float f4t_voltage = 3.3f;` |

---

## 3. Detailed Rules & Code Examples

### Rule 1: Comment Style
**Rule:** `//` comment style must not be used. Use C-style `/* ... */` comments only.
```c
/* Non-compliant */
// TODO : Test this function

/* Compliant */
/* TODO : Test this function */
```

### Rule 2: Braces Placement
**Rule:** `{` and `}` should not be on the same statement line.
```c
/* Non-compliant */
if (u1t_x == 2) { u1t_y = 4; }

/* Compliant */
if (u1t_x == 2) {
    u1t_y = 4;
}

/* Exception: '} else {' is permitted on the same line */
if (u1t_x == 2) {
    u1t_y = 4;
} else {
    u1t_y = 2;
}
```

### Rule 3: Spaces Around Operators
**Rule:** It should have one space between operand and operator.
```c
/* Non-compliant */
u1t_x = (u1t_y+u1t_x)/u1t_a;

/* Compliant */
u1t_x = (u1t_y + u1t_x) / u1t_a;
```

### Rule 4: One Assignment Per Line
**Rule:** Use one assignment per one line.
```c
/* Non-compliant */
u1t_x = u1t_a; u1t_y = u1t_a; u1t_z = u1t_c;

/* Compliant */
u1t_x = u1t_a;
u1t_y = u1t_b;
u1t_z = u1t_c;
```

### Rule 5: Avoid Magic Integer Constants
**Rule:** Hard-coded "magic" integer constants should be best avoided. Define constants with descriptive names.
```c
/* Non-compliant */
u1t_x = 63 - u1t_a;

/* Compliant */
#define FACTOR_A    63U
u1t_x = FACTOR_A - u1t_a;

const uint8_t FACTOR_B = 43U;
u1t_y = FACTOR_B - u1t_a;
```

### Rule 6: Avoid Ternary Operator
**Rule:** Ternary operator `? :` should be best avoided. Use explicit `if-else` blocks instead.
```c
/* Non-compliant */
u1t_x = (u1t_a == 0U) ? u1t_y : u1t_z;

/* Compliant */
if (u1t_a == 0U) {
    u1t_x = u1t_y;
} else {
    u1t_x = u1t_z;
}
```

### Rule 7: No Unreachable Code
**Rule:** A project shall not contain unreachable code (code after return, break, or goto).
```c
/* Non-compliant */
uint8_t u1_getConvFactor(void) {
    const uint8_t CONVFACTORA = 104U;
    return CONVFACTORA;
    const uint8_t CCONVFACTORB = 94U; /* Non-compliant: Unreachable Code */
}

/* Compliant */
uint8_t u1_getConvFactor(void) {
    const uint8_t CONVFACTORA = 104U;
    return CONVFACTORA;
}
```

### Rule 8: No Unused Object Definitions
**Rule:** A project should not contain unused object definitions (variables, static functions, typedefs).
```c
/* An object is unused if the definition and any declarations can be removed, and the program still compiles. */
```

### Rule 9: No Octal Constants
**Rule:** Octal constants shall not be used. Never prefix numbers with leading zero `0`.
```c
/* Non-compliant */
uint8_t u1t_val = 012; /* 012 in octal = 10 decimal -> Confusing & Prohibited */

/* Compliant */
uint8_t u1t_val = 10U;
```

### Rule 10: "u" or "U" Suffix on Unsigned Constants
**Rule:** A `u` or `U` suffix shall be applied to all integer constants that are represented in an unsigned type.
```c
/* Compliance Table */
/* Constant      Type        Compliance    */
/* 32767         int16_t     Compliant     */
/* 0x7fff        int16_t     Compliant     */
/* 32768         uint16_t    Non-compliant */
/* 32768U        uint16_t    Compliant     */
```

### Rule 11: Function Prototypes with Named Parameters
**Rule:** Function types shall be in prototype form with named parameters. If no parameters, explicitly specify `(void)`.
```c
/* Non-compliant */
static uint16_t u2_func1();          /* Missing (void) */
uint8_t u1_func2(uint8_t);           /* Missing parameter name */

/* Compliant */
extern uint32_t u4_func3(void);      /* Explicit 0 parameters */
int16_t s2_func3(uint16_t u2_x);     /* Parameter has clear name */
```

### Rule 12: Initialize Automatic Storage Variables Before Read
**Rule:** The value of an object with automatic storage duration (local stack variable) shall not be read before it has been set.
```c
/* Non-compliant */
int32_t u4_func5(int32_t s4t_x) {
    uint32_t u4t_y;
    int32_t s4t_result; 
    if (u4t_y == 0U) { /* Non-compliant: u4t_y has not been initialized */
        s4t_result = (s4t_x * s4t_x);
    } else {
        s4t_result = s4t_x;
    }
    return s4t_result;
}

/* Compliant */
int32_t u4_func5(int32_t s4t_x) {
    uint32_t u4t_y = 0U; /* Explicitly initialized */
    int32_t s4t_result; 
    if (u4t_y == 0U) {
        s4t_result = (s4t_x * s4t_x);
    } else {
        s4t_result = s4t_x;
    }
    return s4t_result;
}
```

### Rule 13: Appropriate Essential Type for Operands
**Rule:** Operands shall not be of an inappropriate essential type.
```c
/* Non-compliant */
u4t_x && u4t_y                 /* Using uint32_t with logical boolean operator */
!s2t_x                         /* Using signed integer with logical NOT */
u4t_x << (u2t_a == u2t_b)      /* Using boolean result as shift count */
u4t_x >> -1                    /* Shift magnitude uses negative/signed type */

/* Compliant */
(u4t_x != 0U) && (u4t_y != 0U)
(s2t_x == 0)
u4t_x << 2U
u4t_x >> 1U
```

### Rule 14: Explicit Operator Precedence
**Rule:** The precedence of operators within expressions should be made explicit by using parentheses.
```c
/* Non-compliant */
u4t_x = u4t_a + u4t_b / u4t_c - u4t_d * u4t_e;

/* Compliant */
u4t_x = u4t_a + ((u4t_b / u4t_c) - (u4t_d * u4t_e));
```

### Rule 15: No Direct Equality Tests on Floating-Point
**Rule:** Floating-point expressions shall not be tested for equality (`==`) or inequality (`!=`). Use a tolerance-based approach.
```c
/* Non-compliant */
if (f4t_a == f4t_b) { /* Due to rounding errors, this may never be true */
    /* Do something */
}

/* Compliant */
#include <math.h>
#define TOLERANCE 0.000001f

if (fabsf(f4t_a - f4t_b) < TOLERANCE) {
    /* Do something */
}
```

### Rule 16: Loop Counters Must Not Be Floating-Point Type
**Rule:** A loop counter shall not have essentially floating type. Use integer loop counters only.
```c
/* Non-compliant */
for (float f4t_cnt = 0.0f; f4t_cnt < 1.0f; f4t_cnt += 0.01f) {
    /* ... */
}

float u4t_idx = 0.0f;
while (u4t_idx < 10.0f) {
    u4t_idx += 0.1f;
}

/* Compliant */
for (uint32_t u4t_cnt = 0U; u4t_cnt < 100U; u4t_cnt++) {
    float f4t_val = (float)u4t_cnt * 0.01f;
    /* ... */
}
```

### Rule 17: Controlling Expressions Must Be Essentially Boolean
**Rule:** The controlling expression of an `if` statement and the controlling expression of an iteration-statement (`while`, `for`) shall have essentially Boolean type.
```c
/* Non-compliant */
if (u4t_x) { /* Integer used directly as condition */
}
while (s2t_y) {
}

/* Compliant */
if (u4t_x != 0U) {
}
while (s2t_y != 0) {
}
```

### Rule 18: Compound Statement Required for Control Bodies
**Rule:** The body of an iteration-statement (`while`, `do...while`, `for`) or a selection-statement (`if`, `else`, `switch`) shall be a compound-statement enclosed in `{ ... }`.
```c
/* Non-compliant */
if (u4t_x < 30U)
    action1();
else
    action2();

while (u4t_y != 20U)
    action3();

/* Compliant */
if (u4t_x < 30U) {
    action1();
} else {
    action2();
}

while (u4t_y != 20U) {
    action3();
}
```

### Rule 19: All `if ... else if` Must End with an `else`
**Rule:** All `if ... else if` constructs shall be terminated with an `else` statement. Even if no action is needed, write an empty `else` block with an explanatory comment.
```c
/* Non-compliant */
if (u4t_x < 10U) {
    action1();
} /* Missing else */

if (u4t_y > 0U) {
    action2();
} else if (u4t_y == 0U) {
    action3();
} /* Missing terminal else */

/* Compliant */
if (u4t_x < 10U) {
    action1();
} else {
    /* No action */
}

if (u4t_y > 0U) {
    action1();
} else if (u4t_y == 0U) {
    action2();
} else {
    /* Do nothing */
}
```

### Rule 20: Unconditional `break` in Every Switch Clause
**Rule:** An unconditional `break` statement shall terminate every switch-clause. Fall-through is only permitted if the case is completely empty.
```c
/* Non-compliant */
switch (u1t_x) {
    case 0U:
        action1();
        break;
    case 1U:
    case 2U:
        action2();
        break;
    case 3U:
        action3(); /* Non-compliant: break omitted */
    case 4U:
        if (action4() > 20U) {
            break; /* Non-compliant: conditional break */
        }
    default:       /* Non-compliant: default must also have break */
}

/* Compliant */
switch (u1t_x) {
    case 0U:
        action1();
        break;
    case 1U: /* Compliant: empty fall-through to group cases */
    case 2U:
        action2();
        break;
    case 3U:
        action3();
        break;
    case 4U:
        action4();
        break;
    default:
        /* Default handler */
        break;
}
```

### Rule 21: Every Switch Statement Must Have a `default` Label
**Rule:** Every `switch` statement shall have a `default` label.
```c
/* Non-compliant */
switch (u1t_x) {
    case 0U:
        action1();
        break;
    case 1U:
        action2();
        break;
    /* Missing default */
}

/* Compliant */
switch (u1t_x) {
    case 0U:
        action1();
        break;
    case 1U:
        action2();
        break;
    default:
        /* Default handling or comment */
        break;
}
```

### Rule 22: `default` Label Position
**Rule:** A `default` label shall appear as either the first or the last switch label of a `switch` statement. Placing it in the middle is prohibited.
```c
/* Non-compliant */
switch (u1t_x) {
    case 0U:
        action1();
        break;
    default: /* Non-compliant: default is between case 0 and case 1 */
        action3();
        break;
    case 1U:
        action2();
        break;
}

/* Compliant */
switch (u1t_x) {
    case 0U:
        action1();
        break;
    case 1U:
        action2();
        break;
    default: /* Compliant: default is the final label */
        action3();
        break;
}
```

---

## 4. Audit & Verification Workflow

When verifying or refactoring C code for compliance with this standard:

1. **Comments:** Search for `//` and convert to `/* ... */`.
2. **Literals:** Check all integer numbers for unsigned `U` suffix (`0U`, `100U`, `0xFFU`). Ensure no leading zero octal literals (`012`).
3. **Conditionals:** Verify that all `if(...)` and `while(...)` conditions evaluate explicitly to boolean (`!= 0`, `== 0`, `!= NULL`).
4. **Braces & Flow:** Ensure every `if`, `else`, `while`, `for` has `{ ... }` on separate lines. Ensure every `if` construct has a terminating `else`.
5. **Switches:** Ensure `default:` exists, is located at the end, and ends with an unconditional `break;`. Ensure all `case` blocks end with `break;`.
6. **Floating Point:** Ensure no floating-point comparisons use `==` or `!=`, and no loop counters are `float`.
