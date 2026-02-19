
tested with matlab 2022 b and uav toolbox
uav toolbox can be added from matlab-apps-get more apps

## 📄 Setting up Mission Planner for UDP MAVLink Bridge

Mission Planner is the primary Ground Control Station (GCS) for the ArduPilot open-source autopilot. This guide focuses on configuring the MAVLink Mirror to broadcast MAVLink data over UDP.

### ⬇️ Download Mission Planner

You can download the official installer for Mission Planner from the ArduPilot documentation site:

* **[Download the Latest Mission Planner Installer (for Windows)](https://ardupilot.org/planner/docs/mission-planner-installation.html#windows-installation)**

---

### 1. 🔍 Verify the Active Connection

The setup assumes your flight controller is already connected and running. Based on the images, the initial **Serial** connection (Type: Serial, Direction: Inbound, Port: COM3, Host/Baud: 115200) is **Started**.

This confirms Mission Planner is **actively listening** for MAVLink data from the flight controller.

---

### 2. ⚙️ Access Advanced Configuration

Navigate to the advanced settings to manage the MAVLink bridging tool.

* Click on the **"SETUP"** tab in the top menu of Mission Planner.
* In the sidebar on the left, click the **"Advanced"** section.

---

### 3. 🔗 Open MAVLink Mirror

The MAVLink Mirror tool replicates the MAVLink stream to an external network location.

* Within the **"Advanced"** options, find and click the **"Mavlink Mirror"** button.
    * This action opens the **SerialOutput - Mavlink** configuration window.

---

### 4. 🌐 Configure the UDP Outbound Stream

This step ensures the connected MAVLink data is sent out to your desired application.

Ensure one row is configured for **Outbound UDP** as follows:

| Field | Setting | Purpose |
| :--- | :--- | :--- |
| **Type** | **UDP** | The network protocol to use. |
| **Direction** | **Outbound** | Sending data *from* Mission Planner. |
| **Port** | **14555** | The local port number the data will be sent to. |
| **Host/Baud** | **127.0.0.1** | The **localhost** IP address, meaning the data goes to an application on the **same computer**. |
| **Write** | **Checked** | Enables sending data. |
| **Go** | **Started** | Confirms the stream is actively running. |

**Action:** If this row is not set up correctly, input the values above. Ensure the **Write** box is checked and click **"Go"** so the status changes to **"Started"**.

---

### 5. 👂 Listen with the External Application

The UDP MAVLink Bridge is now running.

* Any application you want to monitor the vehicle data with (e.g., a custom script, a simulator, or another GCS) must be configured to listen for **Inbound UDP** data on the address **127.0.0.1** and port **14555**.