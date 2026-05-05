import time

from env.env import MultiUAVEnv
import pybullet as p

def main():
    env = MultiUAVEnv(
        gui=True,
        num_drones=1,
        num_people=3,
        min_people=3,
        max_people=3,
        episode_steps=300,
        max_groups=2,
        num_group_regions=4,
    )

    obs = env.reset()
    # Add these lines right after reset:

    '''
    print("Initial observation shapes:")
    print("grid:", obs[0]["grid"].shape)
    print("local:", obs[0]["local"].shape)
    print("Episode group info:", env.episode_group_info)
    '''
    try:
        done = False
        step = 0

        while not done:
            action = [(0.2, 0.0, 0.0, 0.12)]

            obs, reward, done, info = env.step(action)

            if step % 20 == 0:
                '''
                print(f"Step {step}")
                print("Reward:", reward)
                print("Reward info:", info.get("reward_info", {}))
                print("Visible IDs:", info.get("visible_ids_per_drone", []))
                print("Num groups:", info.get("num_groups"))
                print("Group centers:", info.get("group_centers"))
                print("Group info:", info.get("group_info"))
                print("-" * 50)
                '''
            time.sleep(0.05)
            step += 1

    finally:
        env.close()


if __name__ == "__main__":
    main()