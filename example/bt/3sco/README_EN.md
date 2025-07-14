# BT 3SCO Example

This project is an example for establishing 3 eSCO links simultaneously. Currently, only 58x supports 3SCO.
- Compilation method is the same as other common projects: scons --board=xxxx -j8

- This project can also compile non-58x boards, but only supports 1 eSCO

- The upper layer uses a custom L2CAP profile. In the btskey main Menu, enter btskey a to access the bt_l2cap_profile Menu

- After entering the bt_l2cap_profile Menu, you need to first enter btskey 1 to register the L2CAP profile before proceeding with subsequent ACL/SCO link establishment

- For the data paths of the 3 eSCO links, currently only the first established eSCO path's data is interfaced with the local audio path. The data from the other two eSCO links established later is sent back to the remote party using loopback mode by default.