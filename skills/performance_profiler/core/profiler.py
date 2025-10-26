"""
Performance Profiling Module

Uses cProfile to profile Python code execution.
"""

import cProfile
import pstats
import time
import functools
from io import StringIO
from pathlib import Path
from typing import Dict, List, Optional, Any, Callable
from dataclasses import dataclass
import logging

logger = logging.getLogger(__name__)


@dataclass
class ProfileResult:
    """Result from profiling a function."""
    function_name: str
    total_time: float
    primitive_calls: int
    total_calls: int
    top_functions: List[Dict[str, Any]]
    profile_file: Optional[str] = None

    def to_dict(self) -> Dict:
        """Convert to dictionary."""
        return {
            'function_name': self.function_name,
            'total_time': self.total_time,
            'primitive_calls': self.primitive_calls,
            'total_calls': self.total_calls,
            'top_functions': self.top_functions,
            'profile_file': self.profile_file
        }


class FunctionProfiler:
    """
    Profile Python function execution using cProfile.
    """

    def __init__(self, output_dir: str = "./profiles"):
        """
        Initialize profiler.

        Args:
            output_dir: Directory to store profile outputs
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.profile_results: List[ProfileResult] = []

        logger.info(f"FunctionProfiler initialized, output dir: {output_dir}")

    def profile_function(
        self,
        func: Callable,
        *args,
        save_profile: bool = True,
        top_n: int = 10,
        **kwargs
    ) -> ProfileResult:
        """
        Profile a single function call.

        Args:
            func: Function to profile
            *args: Function arguments
            save_profile: Save profile to file
            top_n: Number of top functions to include
            **kwargs: Function keyword arguments

        Returns:
            ProfileResult with profiling data
        """
        profiler = cProfile.Profile()

        logger.info(f"Profiling function: {func.__name__}")

        # Profile the function
        profiler.enable()
        start_time = time.time()

        try:
            result = func(*args, **kwargs)
            success = True
        except Exception as e:
            logger.error(f"Error during profiling: {e}")
            success = False
            result = None
        finally:
            profiler.disable()

        total_time = time.time() - start_time

        # Analyze results
        stats = pstats.Stats(profiler)
        stats.sort_stats('cumulative')

        # Get top functions
        s = StringIO()
        stats.stream = s
        stats.print_stats(top_n)
        stats_output = s.getvalue()

        # Parse top functions
        top_functions = self._parse_stats(stats_output, top_n)

        # Save profile if requested
        profile_file = None
        if save_profile:
            profile_file = str(
                self.output_dir / f"{func.__name__}_{int(time.time())}.prof"
            )
            profiler.dump_stats(profile_file)
            logger.info(f"Profile saved to: {profile_file}")

        profile_result = ProfileResult(
            function_name=func.__name__,
            total_time=total_time,
            primitive_calls=stats.prim_calls if hasattr(stats, 'prim_calls') else 0,
            total_calls=stats.total_calls,
            top_functions=top_functions,
            profile_file=profile_file
        )

        self.profile_results.append(profile_result)

        return profile_result

    def profile_operations(
        self,
        target_operations: List[Dict[str, Any]],
        iterations: int = 10,
        warmup_iterations: int = 3
    ) -> Dict[str, Any]:
        """
        Profile multiple operations with iterations.

        Args:
            target_operations: List of operations to profile
                              [{'func': callable, 'args': tuple, 'kwargs': dict}]
            iterations: Number of iterations to run
            warmup_iterations: Number of warmup runs

        Returns:
            Aggregated profiling results
        """
        logger.info(
            f"Profiling {len(target_operations)} operations "
            f"with {iterations} iterations ({warmup_iterations} warmup)"
        )

        results = []

        for operation in target_operations:
            func = operation['func']
            args = operation.get('args', ())
            kwargs = operation.get('kwargs', {})

            # Warmup runs
            logger.debug(f"Warmup: {func.__name__}")
            for _ in range(warmup_iterations):
                try:
                    func(*args, **kwargs)
                except Exception as e:
                    logger.warning(f"Warmup error: {e}")

            # Actual profiling runs
            times = []
            for i in range(iterations):
                start = time.time()
                try:
                    func(*args, **kwargs)
                    times.append(time.time() - start)
                except Exception as e:
                    logger.error(f"Iteration {i} error: {e}")

            if times:
                avg_time = sum(times) / len(times)
                min_time = min(times)
                max_time = max(times)
                std_dev = self._calculate_std_dev(times)

                results.append({
                    'operation': func.__name__,
                    'iterations': len(times),
                    'avg_time_ms': avg_time * 1000,
                    'min_time_ms': min_time * 1000,
                    'max_time_ms': max_time * 1000,
                    'std_dev_ms': std_dev * 1000
                })

        return {
            'operation_metrics': results,
            'total_operations': len(results),
            'total_duration': sum(r['avg_time_ms'] for r in results)
        }

    def _parse_stats(self, stats_output: str, top_n: int) -> List[Dict[str, Any]]:
        """Parse cProfile stats output."""
        lines = stats_output.split('\n')
        top_functions = []

        # Skip header lines
        data_started = False
        for line in lines:
            if 'ncalls' in line:
                data_started = True
                continue

            if data_started and line.strip():
                parts = line.split()
                if len(parts) >= 6:
                    try:
                        top_functions.append({
                            'ncalls': parts[0],
                            'tottime': float(parts[1]),
                            'percall': float(parts[2]) if parts[2] != '0.000' else 0.0,
                            'cumtime': float(parts[3]),
                            'function': ' '.join(parts[5:])
                        })

                        if len(top_functions) >= top_n:
                            break
                    except (ValueError, IndexError):
                        continue

        return top_functions

    def _calculate_std_dev(self, values: List[float]) -> float:
        """Calculate standard deviation."""
        if len(values) < 2:
            return 0.0

        mean = sum(values) / len(values)
        variance = sum((x - mean) ** 2 for x in values) / (len(values) - 1)
        return variance ** 0.5

    def get_profile_summary(self) -> Dict[str, Any]:
        """Get summary of all profiling results."""
        return {
            'total_profiles': len(self.profile_results),
            'profiles': [result.to_dict() for result in self.profile_results]
        }

    def clear_results(self) -> None:
        """Clear all profiling results."""
        self.profile_results.clear()
        logger.info("Cleared profiling results")


def profile_decorator(
    output_dir: str = "./profiles",
    save_profile: bool = True
):
    """
    Decorator to profile a function.

    Usage:
        @profile_decorator(output_dir="./my_profiles")
        def my_function():
            # Function code
            pass
    """
    def decorator(func: Callable):
        profiler = FunctionProfiler(output_dir=output_dir)

        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            result = profiler.profile_function(
                func,
                *args,
                save_profile=save_profile,
                **kwargs
            )

            # Log profiling summary
            logger.info(
                f"[PROFILE] {func.__name__}: {result.total_time:.3f}s, "
                f"{result.total_calls} calls"
            )

            # Return original function result
            return func(*args, **kwargs)

        return wrapper
    return decorator


# Export public API
__all__ = [
    'FunctionProfiler',
    'ProfileResult',
    'profile_decorator'
]
