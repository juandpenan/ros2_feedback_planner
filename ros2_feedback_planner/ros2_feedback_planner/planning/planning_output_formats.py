"""Defines output formats for planning results."""

from pydantic import BaseModel, Field


class ActionFuturePrecondition(BaseModel):
    """Represents an action and its associated future preconditions."""

    action: str = Field(..., description="The action command string, e.g., 'move_forward(1.5)'")
    future_preconditions: list[str] = Field(
        default_factory=list,
        description='list of predicted future preconditions that must hold'
    )


class ActionPrecondition(BaseModel):
    """Represents an action and its associated future preconditions."""

    action: str = Field(..., description="The action command string, e.g., 'move_forward(1.5)'")
    preconditions: list[str] = Field(
        default_factory=list,
        description='list of preconditions that must hold'
    )


class ForecastPlan(BaseModel):
    """Represents a basic plan with a reason and a list of actions."""

    reason: str
    plan: list[str]
    action_and_preconditions: ActionFuturePrecondition = Field(
        ...,
        alias='feedback_input',
        description=(
            'The first action from the plan and its current + future preconditions'
        )
    )


class DoReMiPlan(BaseModel):
    """Represents a basic plan with a reason a list of actions and preconditions."""

    reason: str
    plan: list[str]
    action_and_preconditions: ActionPrecondition = Field(
        ...,
        alias='feedback_input',
        description=(
            'The first action from the plan and its current + preconditions'
        )
    )


class MonologuePlan(BaseModel):
    """Represents a monologue plan containing the next action and its reason."""

    next_action: list[str] = Field(
        ...,
        alias='plan',
        description=(
            'the predicted next action formatted as plan with one action'
        )
    )
    action: str = Field(...,
                        alias='feedback_input',
                        description='The first action from the plan')
