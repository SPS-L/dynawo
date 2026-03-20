#!/usr/bin/env bash
# fix_gcc13_includes.sh
set -euo pipefail

SOURCES="dynawo/sources"

add_include() {
  local file="$1"
  local header="$2"
  grep -qF "#include ${header}" "$file" && return 0
  python3 - "$file" "$header" << 'PYEOF'
import sys, re
path, header = sys.argv[1], sys.argv[2]
with open(path) as f:
    content = f.read()
# Find position just after the last #include line
matches = list(re.finditer(r'^#include\s+[<"][^\n]+', content, re.MULTILINE))
if not matches:
    print(f"  SKIP (no includes found): {path}", file=sys.stderr)
    sys.exit(0)
pos = matches[-1].end()
new_content = content[:pos] + f'\n#include {header}' + content[pos:]
with open(path, 'w') as f:
    f.write(new_content)
print(f"  + {header} -> {path}")
PYEOF
}

echo "=== Fixing missing <memory> ==="
grep -rl "std::shared_ptr\|std::unique_ptr\|std::make_shared\|std::make_unique\|std::weak_ptr" \
  "$SOURCES" | xargs grep -L "#include <memory>" \
  | while read -r f; do add_include "$f" "<memory>"; done

echo "=== Fixing missing <algorithm> ==="
grep -rl "std::find\|std::sort\|std::copy\|std::fill\|std::min\b\|std::max\b\|std::remove\b\|std::transform\|std::for_each\|std::count\b" \
  "$SOURCES" | xargs grep -L "#include <algorithm>" \
  | while read -r f; do add_include "$f" "<algorithm>"; done

echo "=== Fixing missing <limits> ==="
grep -rl "std::numeric_limits" \
  "$SOURCES" | xargs grep -L "#include <limits>" \
  | while read -r f; do add_include "$f" "<limits>"; done

echo "=== Fixing missing <cmath> ==="
grep -rl "std::isnan\|std::isinf\|std::isfinite\|std::abs\b\|std::pow\|std::sqrt\|std::floor\|std::ceil\|std::fabs\|std::exp\b\|std::log\b" \
  "$SOURCES" | xargs grep -L "#include <cmath>" \
  | while read -r f; do add_include "$f" "<cmath>"; done

echo "=== Fixing missing <string> ==="
grep -rl "std::string\b\|std::to_string\|std::stoi\|std::stod\|std::stof\b" \
  "$SOURCES" | xargs grep -L "#include <string>" \
  | while read -r f; do add_include "$f" "<string>"; done

echo ""
echo "All done. Commit the changes, then:"
echo "  ./myEnvDynawo.sh build-dynawo"
