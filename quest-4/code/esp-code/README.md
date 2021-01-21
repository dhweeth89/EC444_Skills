# esp-code Readme

This folder contains the code for the fobs.

## How to use
Each `voting.c` file requires specific IP addresses - namely, the IPs of each of the other Fobs at a particular site. `HOST_IP_ADDR_`, `HOST_IP_ADDR_2`, and `LEADER_IP_ADDR` must be changed accordingly.

Then, flash the correct project to its corresponding esp. Once all are flashed, the boards must be reset synchronously using the onboard reset buttons.

The election will then run for about 10 seconds, after which a leader will be chosen and voting can begin. Votes are sent to the server in `js-code`.

If the leader fob goes down, a new leader is chosen within 45 seconds and voting can continue.

## Notes
The code for these fobs were written under the assumption that local elections and leaders were acceptable for the Quest. This assumption was confirmed by Professor Little via email. As such, every site will have its own leader, and each local leader sends vote data to one specific server.

The minimum required number of fobs is two, but this is also under the assumption that every fob has the same hardware. In reality, this is not the case; there is a maximum of two sets of IR (voting) hardware at each site, but a maximum of three fobs. This means that if a fob with the voting hardware goes down, voting can no longer happen at that site until it comes back online.
