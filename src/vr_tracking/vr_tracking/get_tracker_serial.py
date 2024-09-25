from track import ViveTrackerModule


def get_tracker_serial():
    vive_tracker = ViveTrackerModule()
    vive_tracker.print_discovered_objects()
    assert vive_tracker.devices, "No tracker connected."
    for index, value in vive_tracker.devices.items():
        print(index + " serial:")
        print(value.get_serial())           
    return True   
    
def main():
    get_tracker_serial()


if __name__ == "__main__":
    main()
