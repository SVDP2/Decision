# Map TF and Relative Localization Authority Notes

이 문서는 아직 구현 결론이 아니라 다음 설계 결정을 위한 기준이다.

## Problem

GPS odom nodes can publish `map -> leader/base_link` and `map -> follower/base_link`.
Relative ESKF publishes `leader/base_link -> follower/base_link`. These three TFs
cannot all be independent authorities at the same time because the TF tree would
have competing paths/parents for the same vehicles.

## Recommended Authority Model

Use leader global pose as the map anchor and relative localization as the follower
pose authority:

- Map/global owner publishes `map -> leader/base_link`.
- Relative ESKF publishes `leader/base_link -> follower/base_link`.
- Follower map pose is derived by composition, not by an independent
  `map -> follower/base_link` TF publisher.
- Follower GPS, ArUco, LiDAR, and IMU are measurements/corrections, not separate
  TF owners.

This makes relative localization affect the follower position in `map` while
keeping one TF path.

## GPS Handling

Leader GPS odom can be the leader map anchor when RTK is good. Follower GPS odom
should normally enter fusion as a measurement or candidate topic, not publish its
own map TF. Through `platoon_bringup`, both GPS odom launches are passed
`publish_base_tf:=false` for this reason.

## Open Design Choice

A future map-aware fusion node should decide whether it lives inside
`relative_localization_eskf` or as a separate selector/fusion package. The key
invariant is unchanged: one TF authority per edge, no independent GPS TF plus
relative TF loops.
