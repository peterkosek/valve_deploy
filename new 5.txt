# git_backup_push.ps1
# Usage (from PowerShell):  .\git_backup_push.ps1 "commit message"
# If no message is given, it uses a timestamp.

param(
  [string]$Message = ""
)

function Fail($msg) {
  Write-Host "ERROR: $msg" -ForegroundColor Red
  exit 1
}

# Make sure we're in a git repo
git rev-parse --is-inside-work-tree *> $null
if ($LASTEXITCODE -ne 0) { Fail "Not inside a git repository. Run this from your repo folder." }

# Optional: show what branch we're on
$branch = (git rev-parse --abbrev-ref HEAD).Trim()

# Stage everything
git add -A
if ($LASTEXITCODE -ne 0) { Fail "git add failed" }

# If nothing to commit, just push (useful if you rebased/merged elsewhere)
$status = (git status --porcelain)
if ([string]::IsNullOrWhiteSpace($status)) {
  Write-Host "No changes to commit. Pushing branch '$branch'..." -ForegroundColor Yellow
  git push
  if ($LASTEXITCODE -ne 0) { Fail "git push failed" }
  Write-Host "Done." -ForegroundColor Green
  exit 0
}

if ([string]::IsNullOrWhiteSpace($Message)) {
  $Message = "backup $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')"
}

# Commit
git commit -m $Message
if ($LASTEXITCODE -ne 0) { Fail "git commit failed (did you forget to set user.name/user.email?)" }

# Push
git push
if ($LASTEXITCODE -ne 0) { Fail "git push failed" }

Write-Host "Pushed '$branch' with message: $Message" -ForegroundColor Green