#!/bin/sh
# Step the clock to GitHub's time before a logger starts.
#
# The Pi has no battery clock and the bench network blocks NTP, so after a
# power cut it boots at the time it last saved and stays behind until htpdate
# catches up - minutes of wrong CSV timestamps. GitHub's HTTPS Date header is
# reachable whenever pushing works. Run as root by the services' ExecStartPre;
# never fails the service, since logging with a slightly-off clock beats not
# logging at all.

for try in 1 2 3; do
    d=$(curl -fsSI --max-time 10 https://github.com 2>/dev/null | tr -d '\r' | sed -n 's/^[Dd]ate: //p')
    [ -n "$d" ] && break
    sleep 5
done
if [ -z "$d" ]; then
    echo "set-clock: could not get the time from github.com, leaving clock as is"
    exit 0
fi

remote=$(date -d "$d" +%s) || exit 0
off=$((remote - $(date +%s)))
# The header only has 1 s resolution, so leave small offsets to htpdate
# rather than jolt the other logger's timestamps on every service start.
if [ "${off#-}" -le 5 ]; then
    echo "set-clock: clock within ${off}s of github.com, not changed"
    exit 0
fi
date -s "@$remote" >/dev/null && echo "set-clock: clock was ${off}s out, set to $(date '+%F %T %Z') from github.com"
exit 0
