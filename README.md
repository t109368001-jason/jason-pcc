# jason-pcc

## Requirements

- Ubuntu:

  ```bash
  sudo apt update
  sudo apt install build-essential cmake libpcap-dev libtbb-dev libpcl-dev
  ```

- Windows (with [vcpkg](https://github.com/microsoft/vcpkg))

  ```shell
  vcpkg.exe install boost-program-options pcl[pcap,visualization] --triplet x64-windows
  cmake -DCMAKE_TOOLCHAIN_FILE=C:/git/vcpkg/scripts/buildsystems/vcpkg.cmake -DVCPKG_TARGET_TRIPLET=x64-windows ..
  ```

