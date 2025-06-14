# Serial TFT Moonraker Adapter

This project provides an adapter for integrating TFT displays with Klipper firmware for 3D printers.

## Installation

1. Clone the repository:
    ```sh
    cd ~
    git clone https://github.com/moonraker-components/serial-tft-adapter.git
    ```
2. Add a link to `tftadapter.py` in the Moonraker components folder:
    ```sh
    ln -s ~/serial-tft-adapter/components/tftadapter.py ~/moonraker/moonraker/components/tftadapter.py
    ```
3. Configure the `tftadapter` in your Moonraker configuration file:
    ```ini
    [tftadapter]
    serial: /dev/ttyS2
    baud: 115200
    ```
4. Enable automatic updates for the Serial TFT Moonraker Adapter by adding the following configuration
to your Moonraker configuration file:
    ```ini
    [update_manager serial-tft-adapter]
    type: git_repo
    primary_branch: main
    path: ~/serial-tft-adapter
    origin: https://github.com/moonraker-components/serial-tft-adapter.git
    managed_services: moonraker
    ```

5. Enable klipper restart with TFT push button:
    ```ini
    [gcode_button LCD_BUTTON_PRESS]
    pin: !opi:gpiochip1/gpio2
    press_gcode:
    RESTART
    ```
6. Restart Moonraker to apply the changes:
    ```sh
    sudo systemctl restart moonraker
    ```

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request.

## License

This project is licensed under the GNU 3.0 License.
