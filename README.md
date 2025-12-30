# Monza Simulator

https://github.com/user-attachments/assets/eb0ec40e-64b0-44ca-83da-6cea9aa8c444

This project attempts to model the physics of the Monza (Euromatic S.A 1979) mechanical game to ultimately try a variety of control strategies.
This platform serves as a good underactuated nonlinear system to benchmark controllers capable of winning the game.

The game rules consists essentially in driving a ball through the different floors until the last and bottom floor, without falling off the platform on its sides.

## Controllers

The only controller implemented here is a bank of MPCs in charge of controlling the platform depending on which floor the ball is currently at.
Each controller constraints different definitions given by the dynamics of the platform.

## License

[MIT License](LICENSE)
