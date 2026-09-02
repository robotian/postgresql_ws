#!/bin/bash
set -euo pipefail

# Use the current workspace path when the container is opened from a different location.
DOCKER_WORKSPACE="${DOCKER_WORKSPACE:-$(pwd -P)}"
DB_NAME="ros2"
USER="postgres"

mkdir -p "$DOCKER_WORKSPACE/clearpath"

setup_file="$DOCKER_WORKSPACE/clearpath/setup.bash"
: > "$setup_file"

# Source ROS 2 setup
echo "source /opt/ros/humble/setup.bash" >> "$setup_file"

# Set ROS domain ID
echo "export ROS_DOMAIN_ID=0" >> "$setup_file"

# Set RMW implementation
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> "$setup_file"

# Unset ROS Discovery Server
echo "unset ROS_DISCOVERY_SERVER" >> "$setup_file"

# Source the custom setup.bash in .bashrc. Ignore duplicates.
source_line="source $DOCKER_WORKSPACE/clearpath/setup.bash"
if ! grep -qxF "$source_line" ~/.bashrc 2>/dev/null; then
    echo "$source_line" >> ~/.bashrc
fi

# PostgreSQL commands to create database and restore from backup
echo "Starting PostgreSQL database..."
service postgresql start || true

# Adding postgis extension to DB
apt-get update
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends ca-certificates gnupg lsb-release
curl -fsSL https://www.postgresql.org/media/keys/ACCC4CF8.asc | gpg --dearmor > /etc/apt/trusted.gpg.d/apt.postgresql.org.gpg
cat <<EOF > /etc/apt/sources.list.d/pgdg.list
# PostgreSQL Global Development Group repository
# This file is managed by the devcontainer setup script.
deb http://apt.postgresql.org/pub/repos/apt $(lsb_release -cs)-pgdg main
EOF

cat <<'EOF' > /etc/apt/preferences.d/pgdg.pref
Package: *
Pin: release o=apt.postgresql.org
Pin-Priority: 500
EOF

apt-get update
apt-get upgrade -y
apt-cache search postgresql-17 | grep postgis || true
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends postgresql-17-postgis-3

# PostgreSQL commands to create database and restore from backup
echo "Creating and restoring PostgreSQL database..."
service postgresql start || true

# Create database ros2 (if not exists)
createdb "$DB_NAME" 2>/dev/null || psql -U postgres -tc "SELECT 1 FROM pg_database WHERE datname = '$DB_NAME';" | grep -q 1 || psql -U postgres -c "CREATE DATABASE $DB_NAME;"

psql -U postgres -tc "SELECT 1 FROM pg_roles WHERE rolname = 'admin';" | grep -q 1 || psql -U postgres -c "CREATE ROLE admin WITH PASSWORD 'Robotlab2019';"

psql -U postgres -c "ALTER ROLE admin WITH LOGIN SUPERUSER CREATEDB REPLICATION;"

# Restore the database from backup if it exists.
if [ -f "$DOCKER_WORKSPACE/db_backups/ros2_backup.backup" ]; then
    pg_restore -U postgres -d "$DB_NAME" "$DOCKER_WORKSPACE/db_backups/ros2_backup.backup" || true
else
    echo "Backup file not found at $DOCKER_WORKSPACE/db_backups/ros2_backup.backup; skipping restore."
fi

echo "Database setup complete!"