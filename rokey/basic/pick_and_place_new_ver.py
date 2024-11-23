import rclpy
import DR_init

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY = 60
ACC = 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

ON, OFF = 1, 0


class RobotController:
    def __init__(self, node):
        self.node = node
        self.VELOCITY = VELOCITY
        self.ACC = ACC
        self.current_block_size = 0
        self.pick_point_list = []
        self.goal_point_list = []

        try:
            import DSR_ROBOT2 as DS
            from DR_common2 import posx, posj
            self.DS = DS
            self.posx = posx
            self.posj = posj
        except ImportError as e:
            self.node.get_logger().error(f"Error importing robot modules: {e}")
            raise e

    def set_gripper(self, state):
        """Gripper control: state can be 'open' or 'close'."""
        if state == "close":
            self.DS.set_digital_output(1, ON)
            self.DS.set_digital_output(2, OFF)
        elif state == "open":
            self.DS.set_digital_output(1, OFF)
            self.DS.set_digital_output(2, ON)
        self.DS.mwait(0.5)

    def calculate_positions(self, pos1, pos2, pos3, pos4, rows, cols, unzip=False):
        """Generate a grid pattern of positions."""
        vertical = pos3[0] - pos1[0]
        height = pos2[1] - pos1[1]
        pos_list = []

        for r in range(rows):
            for c in range(cols):
                new_pos = [
                    pos1[0] + (vertical * r / (rows - 1) if rows > 1 else 0),
                    pos1[1] + (height * c / (cols - 1) if cols > 1 else 0),
                    pos1[2], pos1[3], pos1[4], pos1[5]
                ]
                pos_list.append(new_pos)

        if unzip:
            reordered_list = []
            for c in range(cols):
                for r in range(rows):
                    reordered_list.append(pos_list[r * cols + c])
            return reordered_list
        return pos_list

    def determine_block_size(self, z_gap):
        """Determine the block size based on Z-gap."""
        if z_gap > 104:
            return 0  # Invalid size
        elif z_gap > 94:
            return 1  # Medium block
        else:
            return 2  # Large block

    def pick_block(self, pick_pos):
        """Pick up a block at a specific position."""
        self.DS.set_ref_coord(self.DS.DR_TOOL)
        pos_start, _ = self.DS.get_current_posx(ref=self.DS.DR_BASE)
        self.DS.task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])

        self.DS.set_desired_force([0, 0, 30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=self.DS.DR_FC_MOD_REL)
        while True:
            if self.DS.check_force_condition(self.DS.DR_AXIS_Z, min=5, ref=self.DS.DR_BASE) == 0:
                self.DS.release_compliance_ctrl()
                pos_end, _ = self.DS.get_current_posx(ref=self.DS.DR_BASE)
                z_gap = pos_start[2] - pos_end[2]
                self.current_block_size = self.determine_block_size(z_gap)
                break

        self.set_gripper("close")
        self.DS.movel([0, 0, 50, 0, 0, 0], v=self.VELOCITY, a=self.ACC, ref=self.DS.DR_TOOL, mod=self.DS.DR_MV_MOD_REL)

    def release_block(self, goal_pos):
        """Release a block at a specific position."""
        self.DS.set_ref_coord(self.DS.DR_TOOL)
        self.DS.task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])

        self.DS.set_desired_force([0, 0, 30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=self.DS.DR_FC_MOD_REL)
        while True:
            if self.DS.check_force_condition(self.DS.DR_AXIS_Z, min=10, ref=self.DS.DR_BASE) == 0:
                self.DS.release_compliance_ctrl()
                self.set_gripper("open")
                break

    def setup_positions(self):
        """Setup pick and goal positions."""
        pos1 = [402.00, -1.00, 150, 0, 180, 0]
        pos2 = [402.00, -103.01, 150, 0, 180, 0]
        pos3 = [500.28, -103.00, 150, 0, 180, 0]
        pos4 = [500.28, 0.01, 150, 0, 180, 0]
        row = 3
        column = 3
        self.pick_point_list = self.calculate_positions(pos1, pos2, pos3, pos4, row, column)

        pos5 = [404.00, 149.62, 150, 0, 180, 0]
        pos6 = [404.00, 49.88, 150, 0, 180, 0]
        pos7 = [502.91, 49.88, 150, 0, 180, 0]
        pos8 = [502.91, 149.88, 150, 0, 180, 0]
        row = 3
        column = 3
        self.goal_point_list = self.calculate_positions(pos5, pos6, pos7, pos8, row, column, unzip=True)

    def execute_task(self):
        """Execute the pick-and-place task."""
        self.DS.set_tool("Tool Weight_2FG")
        self.DS.set_tcp("2FG_TCP")
        self.setup_positions()
        self.set_gripper("open")
        check_goal_list = [False] * len(self.pick_point_list)

        for i, pick_pos in enumerate(self.pick_point_list):
            print("잡기 시작")
            self.DS.movel(pick_pos, v=self.VELOCITY, a=self.ACC, ref=self.DS.DR_BASE)
            self.set_gripper("close")
            self.pick_block(pick_pos)
            print("잡기 끝")
            self.DS.movel(pick_pos, v=self.VELOCITY, a=self.ACC, ref=self.DS.DR_BASE)

            goal_index = self.current_block_size * 3  # Example logic
            while check_goal_list[goal_index]:
                goal_index += 1
            print("놓기 시작")
            self.DS.movel(self.goal_point_list[goal_index], v=self.VELOCITY, a=self.ACC, ref=self.DS.DR_BASE)
            self.release_block(self.goal_point_list[goal_index])
            print("놓기 끝")
            self.DS.movel(self.goal_point_list[goal_index], v=self.VELOCITY, a=self.ACC, ref=self.DS.DR_BASE)
            check_goal_list[goal_index] = True
            print(f"{i+1}회차 끝")

def main():
    rclpy.init()
    node = rclpy.create_node("stack_block")
    controller = RobotController(node)

    try:
        controller.execute_task()
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
