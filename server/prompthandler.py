from __future__ import annotations
from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Any, Literal
from mcp.types import (
    Prompt,
    PromptArgument,
    PromptMessage,
    GetPromptResult,
)
from mcp.types import TextContent

@dataclass
class ArgSpec:
    name: str
    description: str = ""
    required: bool = True
    type: Literal["string", "boolean", "number"] = "string"
    enum: list[Any] | None = None
    default: Any = None

@dataclass
class BasePromptHandler:
    """
    Implement prompts by specifying:
      - name, description
      - args: list of ArgSpec
      - messages_template: list of (role, text_template) tuples
        where text_template can use `{var}` placeholders.
    """
    name: str
    description: str
    args: List[ArgSpec] = field(default_factory=list)
    messages_template: List[Tuple[str, str]] = field(default_factory=list)

    def get_prompt_description(self) -> Prompt:
        return Prompt(
            name=self.name,
            description=self.description,
            arguments=[
                PromptArgument(
                    name=a.name,
                    description=a.description,
                    required=a.required,
                )
                for a in self.args
            ],
        )

    def render(self, arguments: Dict[str, str] | None) -> GetPromptResult:
        arguments = arguments or {}

        # Validate required args + fill defaults for optional
        missing = [a.name for a in self.args if a.required and a.name not in arguments]
        if missing:
            raise ValueError(
                f"Missing required argument(s): {', '.join(missing)} "
                f"for prompt '{self.name}'"
            )

        for a in self.args:
            if not a.required and a.name not in arguments and a.default is not None:
                arguments[a.name] = a.default

        # Render each message with simple str.format
        messages: List[PromptMessage] = []
        for role, template in self.messages_template:
            try:
                text = template.format(**arguments)
            except KeyError as e:
                raise ValueError(
                    f"Template for '{self.name}' references missing arg: {e}"
                ) from e
            messages.append(
                PromptMessage(
                    role=role,
                    content=TextContent(type="text", text=text),
                )
            )

        return GetPromptResult(
            description=self.description,
            messages=messages,
        )
