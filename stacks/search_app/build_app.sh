#!/bin/bash

# Path to Docker daemon configuration file
CONFIG_FILE="/etc/docker/daemon.json"

# Registry to add as an insecure registry
INSECURE_REGISTRY="192.168.2.1:5000"

# Function to install jq
install_jq() {
    echo "Attempting to install jq..."
    if command -v apt-get &> /dev/null; then
        sudo apt-get update
        sudo apt-get install -y jq
    elif command -v yum &> /dev/null; then
        sudo yum install -y jq
    elif command -v dnf &> /dev/null; then
        sudo dnf install -y jq
    else
        echo "Package manager not recognized. Please install jq manually."
        exit 1
    fi
}

# Check if jq is installed, if not, install it
if ! command -v jq &> /dev/null; then
    install_jq
fi

# Create an empty JSON file if it doesn't exist
if [ ! -f "$CONFIG_FILE" ]; then
    echo "{}" > "$CONFIG_FILE"
fi

# Check if the insecure-registries key exists and contains the desired value
if ! jq -e --arg INSECURE_REGISTRY "$INSECURE_REGISTRY" '.["insecure-registries"] | contains([$INSECURE_REGISTRY])' "$CONFIG_FILE" > /dev/null; then
    # Update or add the insecure-registries key with the desired value
    jq --arg INSECURE_REGISTRY "$INSECURE_REGISTRY" 'if .["insecure-registries"] then .["insecure-registries"] += [$INSECURE_REGISTRY] else .["insecure-registries"] = [$INSECURE_REGISTRY] end' "$CONFIG_FILE" > "${CONFIG_FILE}.tmp" && mv "${CONFIG_FILE}.tmp" "$CONFIG_FILE"
    echo "Updated $CONFIG_FILE with insecure-registries: $INSECURE_REGISTRY"
else
    echo "No update needed. $CONFIG_FILE already contains $INSECURE_REGISTRY."
fi

# Restart Docker daemon to apply changes
echo "Restarting Docker to apply changes..."
sudo systemctl restart docker


# Build your Docker image
docker build -t 192.168.2.1:5000/search:latest .

# Push your Docker image
# docker push search:latest
docker push 192.168.2.1:5000/search:latest