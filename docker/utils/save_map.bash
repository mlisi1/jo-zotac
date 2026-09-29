save_map() {
    if [ $# -eq 0 ]; then
        echo "Usage: save_map <name>"
        return 1
    fi
    
    local name="$1"
    local timestamp=$(date +%Y%m%d_%H%M%S)
    local source_path="/tmp/dump"  # Change this to your actual map path
    local dest_dir="/saved_maps"
    local dest_path="${dest_dir}/${timestamp}_${name}"
    
    # Create destination directory if it doesn't exist
    mkdir -p "$dest_dir"
    
    # Copy the file/directory
    cp -r "$source_path" "$dest_path"
    
    if [ $? -eq 0 ]; then
        echo "Map saved to: $dest_path"
    else
        echo "Error: Failed to save map"
        return 1
    fi
}