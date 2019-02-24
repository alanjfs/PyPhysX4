import PhysX4 as px

v = px.PxVec3(2, 2, 2)
v *= 0.5
assert v.x == 1.0

tm = px.PxTransform(px.PxVec3(1, 1, 1))
assert tm.p.x == 1

tm2 = tm.transform(px.PxTransform(px.PxVec3(2, 2, 2)))
assert tm2.p.x == 3
