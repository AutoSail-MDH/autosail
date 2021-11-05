# Global LAN using Zerotier

The following manual will describe how to setup communication between boat and land through 4G cellular network, see [fig 1](#Global-LAN/network-autosail.png).

Follow attached guide to create an account and set up a Zerotier Network and adding clients and server, see "ZeroTier configuration - Teltonika Networks Wiki.html".

![Autosail Network using Zerotier](SOF.09/network-autosail.png)

## Join a Network on Linux PC

### 1. Install zerotier-cli

The following curl command will install zerotier if used in a linux terminal

    > curl -s 'https://pgp.mit.edu/pks/lookup?op=get&search=0x1657198823E52A61' | gpg --import && \ if z=$(curl -s 'https://install.zerotier.com/' | gpg); then echo "$z" | sudo bash; fi

### 2. Join Zerotier network on PC

1. The network_id can be found at [https://my.zerotier.com/](https://my.zerotier.com/)

2. Then use the following command in terminal

    > zerotier-cli join <network_id>

Done
