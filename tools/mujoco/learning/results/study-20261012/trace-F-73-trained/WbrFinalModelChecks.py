"""验证本批四份独立从零训练模型确实自然完成全部采样和优化循环。"""

def require(value, message):
    if not value:
        raise RuntimeError(message)


def verify_final_training(model, policy, run, status, algorithm, expected_steps=262144):
    require(status["status"] == "completed", "训练尚未完成")
    require(status.get("stop_reason") is None and policy.get("stop_reason") is None,
            "停止请求或期限中断的模型不能视作自然完成")
    require(status["steps"] == policy["training_steps"] == model.num_timesteps == expected_steps,
            "最终采样步数不符")
    require(run["initial_steps"] == policy["initial_steps"] == 0
            and run["resume"] is None and policy["resume"] is None,
            "本批比较只接受从零完整训练的最终模型；暂停旧尝试另行保留")
    require(model.n_steps == algorithm["n_steps"] and model.n_envs == policy["envs"]
            and model.n_epochs == algorithm["n_epochs"] and model.batch_size == algorithm["batch_size"],
            "实际批次配置改变")
    batch = model.n_steps * model.n_envs
    require(expected_steps % batch == 0, "固定预算不对齐训练批次")
    require(model._total_timesteps == expected_steps and model._current_progress_remaining == 0.0,
            "实际模型尚未完成最后一批训练循环")
    batches = expected_steps // batch
    # 原算法按KL阈值提前结束某批优化，属于登记行为，不能强制每批做满6轮。
    # _n_updates是优化轮次计数，不是实际梯度更新次数。
    require(batches <= model._n_updates <= batches * model.n_epochs, "实际优化轮次计数异常")
    require(model.target_kl == algorithm["target_kl"], "原优化提前停止阈值改变")
    return dict(natural_budget_completion=True, sampled_steps=model.num_timesteps,
        total_timesteps=model._total_timesteps, progress_remaining=model._current_progress_remaining,
        rollout_steps=batch, rollout_batches=batches, batch_size=model.batch_size, actual_epoch_counter=model._n_updates,
        maximum_epoch_counter=batches * model.n_epochs, target_kl=model.target_kl,
        note="以自然完成标记和循环进度检查最后一批已处理；轮次计数允许原KL阈值提前结束，不等于梯度更新次数。")
