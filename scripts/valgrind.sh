#!/bin/bash
# Valgrind leak checks on a NON-static build (crt-static defeats memcheck's malloc interception).
set -u
cd "$(dirname "$0")/.."
command -v valgrind >/dev/null || { echo "valgrind not found"; exit 1; }
RUSTFLAGS="-D warnings" cargo build --quiet --target-dir target/valgrind || exit 1
BIN=target/valgrind/debug/ft_kalman
SIM=./imu-sensor-stream-linux
VG="valgrind --leak-check=full --show-leak-kinds=definite,indirect,possible --errors-for-leak-kinds=definite,indirect,possible --track-fds=yes --error-exitcode=99 -q"
fail=0
check() { # name expected_exit actual_exit
  if [ "$3" -eq 99 ] || [ "$3" -ne "$2" ]; then echo "FAIL $1 (exit $3, expected $2)"; fail=1; else echo "OK   $1"; fi
}
$VG $BIN -h >/dev/null;            check "help" 0 $?
$VG $BIN --bogus >/dev/null 2>&1;  check "bad args" 2 $?
$VG $BIN -p 1 >/dev/null 2>&1 & # no simulator: 30s timeout
wait $!;                           check "no simulator" 1 $?
port=4300
for opts in "" "-m -d" "-m -d -v /tmp/vg_visual.html"; do
  port=$((port+1))
  $SIM -s 42 -d 1 -p $port --debug >/dev/null 2>&1 &
  simpid=$!
  sleep 2
  $VG $BIN -p $port $opts >/dev/null; check "run [$opts]" 0 $?
  kill $simpid 2>/dev/null; wait $simpid 2>/dev/null
done
rm -f /tmp/vg_visual.html
VGT="$VG --suppressions=scripts/valgrind.supp"
for t in matrix vector noise utils kalman tracker; do
  exe=$(RUSTFLAGS="-D warnings" cargo test --quiet --target-dir target/valgrind --test $t --no-run --message-format=json 2>/dev/null | grep "\"name\":\"$t\"" | grep -o '"executable":"[^"]*"' | tail -1 | cut -d'"' -f4)
  [ -n "$exe" ] || continue
  timeout 180 $VGT "$exe" >/dev/null 2>&1; rc=$?
  # heavy numeric tests are 20-50x slower under valgrind: a timeout is a skip, not a failure
  if [ $rc -eq 124 ]; then echo "SKIP unit $t (too slow under valgrind)"; else check "unit $t" 0 $rc; fi
done
exit $fail
