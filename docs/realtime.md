# Real time

`pilotaro_main` does everything in a single thread, at `frequency` Hz (250 by default): it
collects messages and service calls and runs the estimator → states → controller → actuators
cycle. With `use_sim_time` the loop follows the simulation clock (`/clock`), so it keeps its
rate in simulated time whatever the real time factor; on a robot it follows the wall clock. Between configure and activate (plugins already loaded, before the vehicle can move)
it applies the `real_time.*` configuration:

| Parameter | Default | Effect |
|---|---|---|
| `real_time.enabled` | `true` | Enables everything below. |
| `real_time.priority` | `80` | `SCHED_FIFO` priority of the main thread (1–99). |
| `real_time.lock_memory` | `true` | `mlockall`: the process takes no page faults. |
| `real_time.cpu` | `-1` | Pins the thread to that core (`-1`: no affinity). |
| `real_time.required` | `false` | `true`: if anything fails, pilotaro does not start. `false`: warns and keeps running without real time. |

Every 10 s the node warns if any cycle took longer than the period.

## Permissions

The user needs to be allowed to request real-time priority and lock unlimited memory. On
this machine it is already set up in `/etc/security/limits.conf`:

```text
juanscelyg    -   rtprio   98
juanscelyg    -   memlock  -1
```

For another user or machine:

1. Add those two lines with the corresponding user. It can also be granted to a group
   (`@realtime - rtprio 98`, `@realtime - memlock -1`), adding the user with
   `sudo usermod -aG realtime <user>`.
2. Log out and back in. Limits are applied at login.
3. Check it:

   ```bash
   ulimit -r   # must be >= real_time.priority (98)
   ulimit -l   # must be "unlimited"
   ```

4. At startup, pilotaro confirms it in the log:

   ```text
   [pilotaro_node]: [real_time] Memory locked
   [pilotaro_node]: [real_time] SCHED_FIFO priority 80
   ```

   Without permissions it prints a `[real_time] ... failed, running without it` warning with
   the cause and the current limit (or a fatal error if `real_time.required: true`).

Notes:

- The priority must be `<= rtprio`. If the limit is lowered, `real_time.priority` must be
  lowered too.
- When launched from a systemd service, `limits.conf` does not apply; use
  `LimitRTPRIO=98` and `LimitMEMLOCK=infinity` in the unit.
- In Docker: `--cap-add=sys_nice --ulimit rtprio=98 --ulimit memlock=-1`.
- For tighter latencies, optionally: a kernel with `preempt=full` (Ubuntu 24.04's is
  `PREEMPT_DYNAMIC`, just add it to the boot command line) or a `PREEMPT_RT` kernel, and an
  isolated core (`isolcpus=3`) used with `real_time.cpu: 3`.

[Back to the README](../README.md)
