"""Benchmark VLM planning and feedback-monitoring latency from package configs."""

import argparse
import asyncio
import csv
import importlib
import json
import re
import statistics
import time
from datetime import datetime
from enum import IntEnum
from pathlib import Path

import PIL.Image
import yaml

PROBABILITY_RE = re.compile(r'"probability_of_failure":\s*"([^"]+)"')


class FailureProbability(IntEnum):
    """Failure probability levels used by the feedback node."""

    VERY_IMPROBABLE = 1
    IMPROBABLE = 2
    NEUTRAL = 3
    LIKELY = 4
    VERY_LIKELY = 5

    @classmethod
    def from_string(cls, value: str):
        mapping = {
            'very improbable': cls.VERY_IMPROBABLE,
            'very_improbable': cls.VERY_IMPROBABLE,
            'improbable': cls.IMPROBABLE,
            'neutral': cls.NEUTRAL,
            'likely': cls.LIKELY,
            'very likely': cls.VERY_LIKELY,
            'very_likely': cls.VERY_LIKELY,
        }
        return mapping.get(value.lower().strip(), cls.NEUTRAL)


def default_config_path() -> Path:
    return (
        Path(__file__).resolve().parents[1]
        / 'config'
        / 'manipulation_planner_config.yaml'
    )


def load_ros_parameters(config_path: Path) -> dict:
    with config_path.open('r', encoding='utf-8') as config_file:
        return yaml.safe_load(config_file)


def load_image(image_path: Path | None) -> PIL.Image.Image:
    if image_path:
        return PIL.Image.open(image_path).convert('RGB')

    # Keeps the script runnable for connectivity/latency smoke tests. Use a
    # real experiment image for manuscript numbers.
    image = PIL.Image.new('RGB', (640, 480), color=(245, 245, 245))
    return image


def get_output_format(format_name: str):
    module = importlib.import_module(
        'ros2_feedback_planner.planning.planning_output_formats'
    )
    return getattr(module, format_name)


def percentile(values: list[float], q: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, round((len(ordered) - 1) * q)))
    return ordered[index]


def summarize(rows: list[dict], fields: list[str]) -> dict:
    valid_rows = [row for row in rows if not row.get('error')]
    summary = {'samples': len(rows), 'valid_samples': len(valid_rows)}
    for field in fields:
        values = [
            float(row[field])
            for row in valid_rows
            if row.get(field) not in (None, '')
        ]
        if not values:
            continue
        summary[f'{field}_mean_s'] = statistics.mean(values)
        summary[f'{field}_std_s'] = statistics.stdev(values) if len(values) > 1 else 0.0
        summary[f'{field}_median_s'] = statistics.median(values)
        summary[f'{field}_p95_s'] = percentile(values, 0.95)
        summary[f'{field}_min_s'] = min(values)
        summary[f'{field}_max_s'] = max(values)
    return summary


def write_csv(path: Path, rows: list[dict]):
    if not rows:
        return
    with path.open('w', newline='', encoding='utf-8') as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def make_client(client_config: dict):
    from ros2_feedback_planner.models.client_base import BaseClient

    return BaseClient(
        vendor=client_config['vendor'],
        api_key_variable=client_config['api_key_variable_name'],
        model_name=client_config['model_name'],
        temperature=float(client_config.get('temperature', 0.0)),
        max_tokens=int(client_config.get('max_tokens', 5000)),
    )


def benchmark_planner(
    config: dict,
    image: PIL.Image.Image,
    iterations: int,
    warmup: int,
) -> list[dict]:
    planner_params = config['planner_node']['ros__parameters']
    planner_type = planner_params['planner_type']
    planner_config = planner_params[planner_type]

    client = make_client(planner_params['llm_client'])
    client.set_system_prompt(planner_config['system_prompt'])
    client.set_output_format(get_output_format(planner_config['output_format']))

    prompt = planner_config['planning_prompt']
    rows = []
    for index in range(iterations + warmup):
        started = time.perf_counter()
        answer = client.generate(prompt, image)
        elapsed = time.perf_counter() - started
        if index >= warmup:
            text = str(answer)
            error = 'empty_response' if answer is None else ''
            rows.append({
                'sample': index - warmup + 1,
                'stage': 'planner_full_response',
                'model': planner_params['llm_client']['model_name'],
                'elapsed_s': elapsed,
                'response_chars': len(text),
                'parsed_probability': '',
                'triggered': '',
                'error': error,
            })
    return rows


def benchmark_feedback_full(
    config: dict,
    image: PIL.Image.Image,
    feedback_input: str,
    iterations: int,
    warmup: int,
) -> list[dict]:
    feedback_params = config['feedback_node']['ros__parameters']
    feedback_type = feedback_params['feedback_type']
    feedback_config = feedback_params[feedback_type]

    client = make_client(feedback_params['llm_client'])
    client.set_system_prompt(feedback_config['system_prompt'])

    prompt = feedback_config['prompt'].replace('{feedback_input}', feedback_input)
    threshold = FailureProbability.from_string(
        feedback_config.get('probability_threshold', 'likely')
    )

    rows = []
    for index in range(iterations + warmup):
        started = time.perf_counter()
        answer = client.generate(prompt, image)
        elapsed = time.perf_counter() - started
        if index < warmup:
            continue

        answer_text = str(answer)
        error = 'empty_response' if answer is None else ''
        probability = ''
        triggered = ''
        match = PROBABILITY_RE.search(answer_text)
        if match:
            probability = match.group(1)
            triggered = FailureProbability.from_string(probability) >= threshold

        rows.append({
            'sample': index - warmup + 1,
            'stage': 'feedback_full_response',
            'model': feedback_params['llm_client']['model_name'],
            'elapsed_s': elapsed,
            'response_chars': len(answer_text),
            'parsed_probability': probability,
            'triggered': triggered,
            'error': error,
        })
    return rows


async def benchmark_feedback_stream(
    config: dict,
    image: PIL.Image.Image,
    feedback_input: str,
    iterations: int,
    warmup: int,
) -> list[dict]:
    feedback_params = config['feedback_node']['ros__parameters']
    feedback_type = feedback_params['feedback_type']
    feedback_config = feedback_params[feedback_type]
    threshold = FailureProbability.from_string(
        feedback_config.get('probability_threshold', 'likely')
    )

    client = make_client(feedback_params['llm_client'])
    client.set_system_prompt(feedback_config['system_prompt'])

    prompt = feedback_config['prompt'].replace('{feedback_input}', feedback_input)
    rows = []

    for index in range(iterations + warmup):
        started = time.perf_counter()
        first_token_s = None
        first_probability_s = None
        decision_s = None
        parsed_probability = ''
        triggered = False
        buffer = ''
        error = ''

        try:
            # Mirrors feedback_server.main_loop(): prompt + image, Gemini stream.
            stream = await client.live_client.aio.models.generate_content_stream(
                model=feedback_params['llm_client']['model_name'],
                contents=[prompt, image],
            )
            async for chunk in stream:
                if not chunk.text:
                    continue
                now = time.perf_counter()
                if first_token_s is None:
                    first_token_s = now - started
                buffer += chunk.text

                match = PROBABILITY_RE.search(buffer)
                if match and first_probability_s is None:
                    first_probability_s = now - started
                    parsed_probability = match.group(1)
                    triggered = (
                        FailureProbability.from_string(parsed_probability) >= threshold
                    )
                    if triggered:
                        decision_s = first_probability_s
                        break
        except Exception as exc:
            error = str(exc)

        total_s = time.perf_counter() - started
        if index < warmup:
            continue

        rows.append({
            'sample': index - warmup + 1,
            'stage': 'feedback_stream',
            'model': feedback_params['llm_client']['model_name'],
            'total_s': total_s,
            'first_token_s': first_token_s if first_token_s is not None else '',
            'first_probability_s': (
                first_probability_s if first_probability_s is not None else ''
            ),
            'decision_s': decision_s if decision_s is not None else '',
            'response_chars': len(buffer),
            'parsed_probability': parsed_probability,
            'triggered': triggered,
            'error': error,
        })

    return rows


def default_feedback_input() -> str:
    return (
        "{'action': 'pick(black)', "
        "'future_preconditions': ['trajectory_clear may fail if the other robot "
        "arm enters the workspace in the next second']}"
    )


def parse_args():
    parser = argparse.ArgumentParser(
        description='Collect VLM latency samples for planner and feedback modules.'
    )
    parser.add_argument(
        '--config',
        type=Path,
        default=default_config_path(),
        help='Planner YAML config path.',
    )
    parser.add_argument(
        '--image',
        type=Path,
        default=None,
        help='Optional RGB image from an experiment frame.',
    )
    parser.add_argument(
        '--feedback-input',
        default=default_feedback_input(),
        help='Current action/preconditions string used in feedback prompt.',
    )
    parser.add_argument(
        '--iterations',
        type=int,
        default=20,
        help='Measured samples per benchmark.',
    )
    parser.add_argument(
        '--warmup',
        type=int,
        default=2,
        help='Warmup calls excluded from summaries.',
    )
    parser.add_argument(
        '--mode',
        choices=['planner', 'feedback-full', 'feedback-stream', 'all'],
        default='all',
        help='Which benchmark to run.',
    )
    parser.add_argument(
        '--output-dir',
        type=Path,
        default=Path('latency_results'),
        help='Directory where CSV and summary JSON files are written.',
    )
    return parser.parse_args()


async def async_main():
    args = parse_args()
    config = load_ros_parameters(args.config)
    image = load_image(args.image)

    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    output_dir = args.output_dir / timestamp
    output_dir.mkdir(parents=True, exist_ok=True)

    all_summaries = {}

    if args.mode in ('planner', 'all'):
        rows = benchmark_planner(config, image, args.iterations, args.warmup)
        write_csv(output_dir / 'planner_latency.csv', rows)
        all_summaries['planner'] = summarize(rows, ['elapsed_s'])

    if args.mode in ('feedback-full', 'all'):
        rows = benchmark_feedback_full(
            config, image, args.feedback_input, args.iterations, args.warmup
        )
        write_csv(output_dir / 'feedback_full_latency.csv', rows)
        all_summaries['feedback_full'] = summarize(rows, ['elapsed_s'])

    if args.mode in ('feedback-stream', 'all'):
        rows = await benchmark_feedback_stream(
            config, image, args.feedback_input, args.iterations, args.warmup
        )
        write_csv(output_dir / 'feedback_stream_latency.csv', rows)
        all_summaries['feedback_stream'] = summarize(
            rows,
            ['total_s', 'first_token_s', 'first_probability_s', 'decision_s'],
        )

    with (output_dir / 'summary.json').open('w', encoding='utf-8') as summary_file:
        json.dump(all_summaries, summary_file, indent=2)

    print(json.dumps(all_summaries, indent=2))
    print(f'Wrote latency results to: {output_dir}')
    if args.image is None:
        print('Note: no --image was provided; use real experiment frames for paper data.')


def main():
    asyncio.run(async_main())


if __name__ == '__main__':
    main()
