"""
Code Analyzer

Main interface for code analysis operations.
"""

import ast
from pathlib import Path
from typing import List, Optional

from .models import (
    FileAnalysis,
    CodebaseAnalysis,
    DependencyGraph,
    DependencyEdge,
    IntegrationPoint,
    DesignPattern,
)
from .python_analyzer import PythonAnalyzer
from .pattern_detector import PatternDetector


class CodeAnalyzer:
    """
    Main interface for code analysis operations.

    Usage:
        analyzer = CodeAnalyzer()
        analysis = analyzer.analyze_codebase("src/")
        patterns = analysis.patterns_found
        integration_points = analysis.integration_points
    """

    def __init__(self):
        self.python_analyzer = PythonAnalyzer()
        self.pattern_detector = PatternDetector()

    def analyze_codebase(
        self,
        root_path: str | Path,
        include_patterns: Optional[List[str]] = None,
        exclude_patterns: Optional[List[str]] = None,
        max_files: Optional[int] = None,
    ) -> CodebaseAnalysis:
        """
        Analyze entire codebase.

        Args:
            root_path: Root directory
            include_patterns: Glob patterns to include (default: ["**/*.py"])
            exclude_patterns: Patterns to exclude
            max_files: Maximum files to analyze

        Returns:
            Complete CodebaseAnalysis
        """
        root = Path(root_path)

        # Default patterns
        if not include_patterns:
            include_patterns = ["**/*.py"]

        if not exclude_patterns:
            exclude_patterns = [
                "**/__pycache__/**",
                "**/.git/**",
                "**/venv/**",
                "**/env/**",
                "**/.venv/**",
                "**/node_modules/**",
            ]

        # Discover files
        files = self._discover_files(root, include_patterns, exclude_patterns, max_files)

        # Analyze each file
        file_analyses = []
        total_lines = 0

        for file_path in files:
            analysis = self.analyze_file(file_path)
            file_analyses.append(analysis)
            total_lines += analysis.total_lines

        # Build dependency graph
        dep_graph = self._build_dependency_graph(file_analyses, root)

        # Collect patterns
        patterns_found = {}
        for analysis in file_analyses:
            for pattern in analysis.patterns:
                if pattern not in patterns_found:
                    patterns_found[pattern] = []
                patterns_found[pattern].append(analysis.file_path)

        # Identify integration points
        integration_points = self._identify_integration_points(file_analyses)

        # Find entry points
        entry_points = self._find_entry_points(file_analyses)

        return CodebaseAnalysis(
            root_path=str(root),
            total_files=len(file_analyses),
            total_lines=total_lines,
            files=file_analyses,
            dependency_graph=dep_graph,
            patterns_found=patterns_found,
            integration_points=integration_points,
            entry_points=entry_points,
        )

    def analyze_file(self, file_path: str | Path) -> FileAnalysis:
        """
        Analyze a single file.

        Args:
            file_path: Path to Python file

        Returns:
            FileAnalysis for the file
        """
        path = Path(file_path)

        # Use Python analyzer
        analysis = self.python_analyzer.analyze_file(path)

        # Detect patterns
        try:
            content = path.read_text(encoding="utf-8")
            tree = ast.parse(content, filename=str(path))
            analysis.patterns = self.pattern_detector.detect_patterns(analysis, tree)
        except (SyntaxError, UnicodeDecodeError):
            # Already handled in python_analyzer
            pass

        return analysis

    def _discover_files(
        self,
        root: Path,
        include: List[str],
        exclude: List[str],
        max_files: Optional[int],
    ) -> List[Path]:
        """Discover Python files to analyze"""
        files = []

        for pattern in include:
            for file_path in root.glob(pattern):
                if not file_path.is_file():
                    continue

                # Check exclusions
                should_exclude = False
                for ex_pattern in exclude:
                    if file_path.match(ex_pattern):
                        should_exclude = True
                        break

                if not should_exclude:
                    files.append(file_path)

                    if max_files and len(files) >= max_files:
                        return files

        return files

    def _build_dependency_graph(
        self, file_analyses: List[FileAnalysis], root: Path
    ) -> DependencyGraph:
        """Build import dependency graph"""
        edges = []

        # Create file path to module name mapping
        file_to_module = {}
        for analysis in file_analyses:
            # Convert file path to module name
            try:
                rel_path = Path(analysis.file_path).relative_to(root)
                module_parts = list(rel_path.parts[:-1])  # Directories

                if rel_path.name != "__init__.py":
                    module_parts.append(rel_path.stem)  # Filename without .py

                module_name = (
                    ".".join(module_parts) if module_parts else rel_path.stem
                )
                file_to_module[analysis.file_path] = module_name
            except ValueError:
                # File not relative to root
                continue

        # Build reverse mapping
        module_to_file = {v: k for k, v in file_to_module.items()}

        # Build edges
        for analysis in file_analyses:
            # Process imports
            for imp in analysis.imports:
                if imp in module_to_file:
                    edges.append(
                        DependencyEdge(
                            from_file=analysis.file_path,
                            to_file=module_to_file[imp],
                            import_type="import",
                            imported_names=[imp],
                        )
                    )

            # Process from imports
            for module, names in analysis.from_imports.items():
                if module in module_to_file:
                    edges.append(
                        DependencyEdge(
                            from_file=analysis.file_path,
                            to_file=module_to_file[module],
                            import_type="from_import",
                            imported_names=names,
                        )
                    )

        return DependencyGraph(edges=edges)

    def _identify_integration_points(
        self, file_analyses: List[FileAnalysis]
    ) -> List[IntegrationPoint]:
        """Identify good integration points"""
        points = []

        for analysis in file_analyses:
            # Base classes are good integration points
            for cls in analysis.classes:
                # Abstract base classes or classes with "Base" in name
                if "Base" in cls.name or "Abstract" in cls.name:
                    points.append(
                        IntegrationPoint(
                            name=cls.name,
                            file_path=analysis.file_path,
                            line_number=cls.line_start,
                            entity_type="class",
                            reason=f"Base class - extend to add new functionality",
                            pattern="Inheritance",
                        )
                    )

                # Classes with Factory pattern
                if "Factory" in cls.name:
                    points.append(
                        IntegrationPoint(
                            name=cls.name,
                            file_path=analysis.file_path,
                            line_number=cls.line_start,
                            entity_type="class",
                            reason="Factory class - add new product types here",
                            pattern="Factory",
                        )
                    )

                # Manager or Client classes
                if "Manager" in cls.name or "Client" in cls.name:
                    points.append(
                        IntegrationPoint(
                            name=cls.name,
                            file_path=analysis.file_path,
                            line_number=cls.line_start,
                            entity_type="class",
                            reason=f"Central {cls.name} - good integration point for new functionality",
                        )
                    )

            # Public functions with "register" or "add" in name
            for func in analysis.functions:
                if func.is_public and any(
                    keyword in func.name.lower()
                    for keyword in ["register", "add", "create", "make"]
                ):
                    points.append(
                        IntegrationPoint(
                            name=func.name,
                            file_path=analysis.file_path,
                            line_number=func.line_start,
                            entity_type="function",
                            reason=f"Registration/creation function - use to add new items",
                        )
                    )

        return points

    def _find_entry_points(self, file_analyses: List[FileAnalysis]) -> List[str]:
        """Find entry point files"""
        entry_points = []

        for analysis in file_analyses:
            file_name = Path(analysis.file_path).name

            # Main files
            if file_name in ["main.py", "__main__.py", "app.py", "cli.py"]:
                entry_points.append(analysis.file_path)

            # __init__.py files (package entry points)
            elif analysis.is_init_file:
                entry_points.append(analysis.file_path)

        return entry_points
