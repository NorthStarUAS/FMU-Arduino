# Fedora 44 not liking Teensy MTP

root# vi /etc/fuse.conf
uncomment: user_allow_other

$ jmtpfs ~/Downloads/tmp/ -o allow_other,nonempty

MTP will be super slow on a teensy 3.6 because it doesn't have much spare CPU
capacity to service MTP requests.
