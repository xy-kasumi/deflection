# Vocabulary

Names for concepts that span code, UI, and conversation. If new behavior doesn't fit, extend the vocab first.

- **chain** — entirety of the connected beams.
- **walker** — virtual thing that "walks" on the chain, basis of explaining beam-local coordinates & `KW_DIR` in [DSL](DSL.md).
- **root beam** — first beam-def.
- **load node** — a point where a force is applied.
- **query node** — a point whose deflection we report.
- **tip** — point on a chain which is natural "end" of the chain. Usually end of last beam; exception: mid of root beam when there's only one beam and `support(both)`.
- **directional** — scalar defined over unit sphere surface. used for representing deflection.
- **lobe** — 3D visualization of a directional.
- **current beam** — the beam-def the editor cursor is in.
- **visual unit** — basis of 3D view object sizing where 1u is still visible. 1u = rod diameter.
