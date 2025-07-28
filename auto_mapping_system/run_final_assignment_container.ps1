docker run `
-e RESOLUTION=1920x1080 `
--name final_assignment `
-p 15900:5900 `
-p 13389:3389 `
-p 9090:9090 `
-p 9876:9876 `
-p 10022:22 `
-p 6080:80 `
--shm-size=64GB `
--privileged `
airobotbook/ros2-desktop-ai-robot-book-humble