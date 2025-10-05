import { Microscope, Rocket } from "lucide-react";
import { useMode } from "@/contexts/ModeContext";
import { Button } from "@/components/ui/button";
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuTrigger,
} from "@/components/ui/dropdown-menu";
import { Tooltip, TooltipContent, TooltipTrigger } from "@/components/ui/tooltip";

export const ModeToggle = () => {
  const { mode, setMode } = useMode();

  return (
    <DropdownMenu>
      <Tooltip>
        <TooltipTrigger asChild>
          <DropdownMenuTrigger asChild>
            <Button variant="outline" size="icon" className="w-9 px-0">
              {mode === "explorer" ? (
                <Rocket className="h-4 w-4" />
              ) : (
                <Microscope className="h-4 w-4" />
              )}
              <span className="sr-only">Toggle mode</span>
            </Button>
          </DropdownMenuTrigger>
        </TooltipTrigger>
        <TooltipContent>
          <p>Current mode: {mode === "explorer" ? "Explorer" : "Researcher"}</p>
        </TooltipContent>
      </Tooltip>
      <DropdownMenuContent align="end">
        <DropdownMenuItem onClick={() => setMode("explorer")}>
          <Rocket className="mr-2 h-4 w-4" />
          <span>Explorer Mode</span>
        </DropdownMenuItem>
        <DropdownMenuItem onClick={() => setMode("researcher")}>
          <Microscope className="mr-2 h-4 w-4" />
          <span>Researcher Mode</span>
        </DropdownMenuItem>
      </DropdownMenuContent>
    </DropdownMenu>
  );
};