# DSL
Deflection tool has tiny DSL to denote serially-connected beams.

example
```
support(both)
mid:horz beam(aluminum rect(W30 H20 T2) L100) mid:force(0.5) end:force(0.5kgf)
right beam(plastic moment(Ix1000 Iy1e6 J1000) L200) mid:force(0.3)
up beam(steel rect(W7 H5) L50) force(0.2)
```

EBNF
```
(* Parser *)
lang = { env-def | beam-def } (* each line cannot contain more than one defs *)

env-def = IDENT paramlist
beam-def = [ loc-spec ] KW_DIR KW_BEAM [ paramlist ] { attachment }
attachment = [ loc-spec ] IDENT paramlist
loc-spec = ( KW_LOC | QUANTITY ) LOC-SEP

paramlist = LP { param } RP
param = QUANTITY | IDENT [ paramlist ]

(* Lexer; lexer ignores whitespaces; tokens must be separated by
  separator LP|RP|LOC-SEP or whitespace *)
LP = "("
RP = ")"
LOC-SEP = ":"

KW_BEAM = "beam"
KW_DIR = "horz" | "right" | "left" | "up" | "down"
KW_LOC = "end" | "mid"
IDENT = /[a-z]+/ (* that is not any of BEAM/DIR/LOC *)
QUANTITY = [ PREFIX ] NUMBER [ UNIT ] (* no space in-between *)
PREFIX = /[A-Z][A-Za-z]*/
NUMBER = (* floating point number *)
UNIT = "N" | "kgf" | "mm4" | "N/mm" | ...
```

semantics
* known `IDENT`:
    * material: `plastic`, `aluminum`, `steel`
    * shape: `rect`, `round`, `moment`
    * attachment: `force`
* Default unit is "mm", "kgf", "mm4"
* `horz` is ambiguous other than for the root beam
* `end` is assumed for omitted loc-spec
* `support` environment defines boundary conditions on the **root beam**
  (the first beam). Subsequent beams attach as appendages — they transmit
  load into the root beam through their attachment point but have no
  boundary conditions of their own.
    * `support(single)`: cantilever — root beam clamped at its start.
    * `support(both)`: fixed-fixed — root beam clamped at both ends, with
      its axis direction left free (so axial strain isn't over-constrained).
      Models the "both ends rigidly bracketed / bolted down" case;
      a floppy multi-pillar frame (slender posts) would need a separate
      model.
