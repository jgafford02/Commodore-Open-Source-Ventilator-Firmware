# Commodore-Open-Source-Ventilator-Firmware

![CoV](https://vandyvents.com/wp-content/uploads/2020/04/System_Labeled-1-1536x925.png)


_**Disclaimer: This code is provided for reference use only.**_ *It was not developed under an ISO-compliant quality systems framework, nor has its function been validated and approved by the Food and Drug Administration. Prior to deployment on an end-use system, to ensure patient safety, the software must undergo substantial validation pursuant to IEC 62304. Use at your own risk.*

This software is meant to provide starter code for others working to develop similar mechanical ventilator designs which operate in open-loop with limit-switch-based feedback. It is by no means representative of a fully validated end-use system, but rather, it implements basic functionality required from a mechanical ventilator (the ability to set and configure BPM and I/E ratios, measure pressure, and alert the user to faults and alarm states). This is an active project, and many aspects are still under development. As such, updates and bug fixes to the software may be posted in the near future. Others working on similar designs may want to modify the software or add features for their own needs.

This code implements two modes of operation that are selectable before bootup. The first is a ventilation mode, where the device is configured to deliver respiratory assistance to patients who are not breathing on their own and require positive pressure assistance. The state machine for this implementation is shown below:

![Ventilation State Diagram](https://vandyvents.com/wp-content/uploads/2020/05/Vent_Diagram.png)

The second mode of operation is a breath-assist mode, where the device is configured to deliver positive airway pressure when the patient attempts an inspiration. The state machine for this implementation is shown below:

![BIPAP State Diagram](https://vandyvents.com/wp-content/uploads/2020/05/BiPAP_Diagram-1.png)

Please go to https://vandyvents.com/ for more details regarding the Commodore Open-Source Ventilator, as well as detailed plans for building your own!
