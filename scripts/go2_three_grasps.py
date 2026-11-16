"""Three distinct processed targets with confirmed StandUp; no hardware access on import."""


def run_three_grasps(run_round, save_progress):
    completed = []
    save_progress(completed)
    for number in range(1, 4):
        result = run_round(number, [r['target_map_xy'] for r in completed])
        if result.get('task_motion_complete') is not True:
            raise RuntimeError('Round did not complete grasp and StandUp; do not count it')
        if result.get('outcome', 'COMPLETED') not in ('COMPLETED', 'MISSED'):
            raise RuntimeError('Unknown target outcome; do not count it')
        completed.append(result)
        save_progress(completed)
    return completed


def progress_snapshot(results):
    """Persist terminal outcomes separately from the number of processed targets."""
    return dict(processed_count=len(results),
                completed_count=sum(r.get('outcome', 'COMPLETED') == 'COMPLETED' for r in results),
                missed_count=sum(r.get('outcome') == 'MISSED' for r in results),
                goal_count=3, completed=results, placement_confirmed=None)
